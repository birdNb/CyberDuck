#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BLE 语音：系统提示音 + conversation_bag 对话短码播放。"""

from __future__ import annotations

import json
import os
import queue
import re
import shutil
import subprocess
import threading
import time
from typing import Callable, Dict, Optional, Tuple, Union

from ble_volume_manager import _audio_env

LogFn = Callable[[str], None]
QueueItem = Union[str, Tuple[str, str, str], None]

PROMPTS = {
    "ble_ready": ("ble_ready.wav", "蓝牙就绪，待连接"),
    "ble_connected": ("ble_connected.wav", "蓝牙已连接"),
    "ble_disconnected": ("ble_disconnected.wav", "蓝牙已断开"),
    "wifi_changed": ("wifi_changed.wav", "WiFi 配置已修改"),
    "wifi_connected": ("wifi_connected.wav", "WiFi 已连接"),
    "wifi_disconnected": ("wifi_disconnected.wav", "WiFi 已断开"),
}

_ASSETS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")
_BAG_DIR = os.path.join(_ASSETS_DIR, "conversation_bag")
_CODE_RE = re.compile(r"^[A-Z][A-Z0-9]{1,31}$")
DIGIT_SEC = 0.5
IP_REPEAT = 2
IP_GAP_SEC = 3.0
BOOT_IP_VOLUME_PCT = 80
IP_ANNOUNCE_COOLDOWN_SEC = 45.0

_default_bag: Optional["ConversationBag"] = None
_bag_lock = threading.Lock()


def normalize_code(code: str) -> str:
    return (code or "").strip().upper()


class ConversationBag:
    """conversation_bag：FFE1 短码（录音文案前5字拼音首字母）→ WAV。"""

    def __init__(
        self,
        bag_dir: Optional[str] = None,
        play_file: Optional[Callable[[str, str], bool]] = None,
        log: LogFn = print,
    ) -> None:
        self._bag_dir = bag_dir or _BAG_DIR
        self._play_file = play_file
        self._log = log
        self._index: Dict[str, Tuple[str, str]] = {}
        self._manifest_mtime: float = 0.0
        self.reload()

    def reload(self) -> int:
        os.makedirs(self._bag_dir, exist_ok=True)
        index: Dict[str, Tuple[str, str]] = {}
        manifest_path = os.path.join(self._bag_dir, "manifest.json")
        if os.path.isfile(manifest_path):
            try:
                with open(manifest_path, encoding="utf-8") as f:
                    data = json.load(f)
                for code, meta in (data.get("items") or {}).items():
                    norm = normalize_code(str(code))
                    if not _CODE_RE.match(norm):
                        continue
                    if not isinstance(meta, dict):
                        meta = {}
                    fname = str(meta.get("file") or f"{norm}.wav")
                    path = os.path.join(self._bag_dir, fname)
                    if os.path.isfile(path):
                        label = str(meta.get("text") or norm)
                        index[norm] = (path, label)
                try:
                    self._manifest_mtime = os.path.getmtime(manifest_path)
                except OSError:
                    self._manifest_mtime = 0.0
            except (OSError, json.JSONDecodeError) as e:
                self._log(f"[conv] manifest 读取失败: {e}")

        try:
            for fname in os.listdir(self._bag_dir):
                if not fname.lower().endswith(".wav"):
                    continue
                code = normalize_code(fname[:-4])
                if not _CODE_RE.match(code) or code in index:
                    continue
                path = os.path.join(self._bag_dir, fname)
                index[code] = (path, code)
        except OSError:
            pass

        self._index = index
        return len(index)

    def _maybe_reload(self) -> None:
        manifest_path = os.path.join(self._bag_dir, "manifest.json")
        if not os.path.isfile(manifest_path):
            return
        try:
            mtime = os.path.getmtime(manifest_path)
        except OSError:
            return
        if mtime != self._manifest_mtime:
            n = self.reload()
            self._log(f"[conv] manifest 已重载，{n} 条语音")

    def has_code(self, code: str) -> bool:
        self._maybe_reload()
        return normalize_code(code) in self._index

    def play(self, code: str) -> bool:
        norm = normalize_code(code)
        self._maybe_reload()
        item = self._index.get(norm)
        if item is None:
            self._log(f"[conv] 未知 code: {norm!r}")
            return False
        path, label = item
        if self._play_file is None:
            self._log(f"[conv] 播放器未就绪: {label}")
            return False
        ok = self._play_file(path, label)
        if not ok:
            self._log(f"[conv] 播放失败: {label} ({norm})")
        return ok


def get_conversation_bag(
    play_file: Optional[Callable[[str, str], bool]] = None,
    log: LogFn = print,
) -> ConversationBag:
    global _default_bag
    with _bag_lock:
        if _default_bag is None:
            _default_bag = ConversationBag(play_file=play_file, log=log)
        else:
            if play_file is not None:
                _default_bag._play_file = play_file
            _default_bag._log = log
        return _default_bag


def conversation_code_exists(code: str) -> bool:
    norm = normalize_code(code)
    if not _CODE_RE.match(norm):
        return False
    return get_conversation_bag().has_code(norm)


class VoicePlayer:
    def __init__(self, log: LogFn = print, assets_dir: Optional[str] = None) -> None:
        self._log = log
        self._assets_dir = assets_dir or _ASSETS_DIR
        self._queue: queue.Queue[QueueItem] = queue.Queue()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._play_cmd = self._detect_play_cmd()
        self._ready_played = False
        self._ready_done = threading.Event()
        self._play_lock = threading.Lock()
        self._current_proc: Optional[subprocess.Popen] = None
        self._last_play_ts: Dict[str, float] = {}
        self._ip_announce_lock = threading.Lock()
        self.conversation = get_conversation_bag(play_file=self.play_file, log=log)

    @staticmethod
    def _detect_play_cmd() -> Optional[list]:
        if shutil.which("pw-play"):
            return ["pw-play"]
        if shutil.which("paplay"):
            return ["paplay", "--volume=65536"]
        if shutil.which("aplay"):
            return ["aplay", "-q"]
        return None

    def start(self) -> None:
        if self._play_cmd is None:
            self._log("[voice] 未找到 pw-play/paplay/aplay，语音提示已禁用")
            return
        missing = [
            name
            for name, (fname, _) in PROMPTS.items()
            if not os.path.isfile(os.path.join(self._assets_dir, fname))
        ]
        if missing:
            self._log(f"[voice] 缺少语音文件: {missing}")
        n = self.conversation.reload()
        self._log(
            f"[voice] 提示音已就绪 ({self._play_cmd[0]}) dir={self._assets_dir}"
        )
        self._log(f"[conv] conversation_bag 已加载 {n} 条（FFE1 发拼音首字母短码）")
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._stop_current_proc()
        self._queue.put(None)
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _stop_current_proc(self) -> None:
        with self._play_lock:
            proc = self._current_proc
            if proc is not None and proc.poll() is None:
                try:
                    proc.terminate()
                    proc.wait(timeout=0.4)
                except (OSError, subprocess.SubprocessError, subprocess.TimeoutExpired):
                    try:
                        proc.kill()
                    except OSError:
                        pass
                finally:
                    self._current_proc = None

    def _drain_queue(self) -> None:
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break

    def play(self, prompt_id: str) -> None:
        if self._play_cmd is None:
            return
        fname = PROMPTS.get(prompt_id, (None, None))[0]
        if not fname:
            return
        path = os.path.join(self._assets_dir, fname)
        if not os.path.isfile(path):
            self._log(f"[voice] 找不到 {fname}")
            return
        self._queue.put(prompt_id)

    def play_file(self, path: str, label: str = "") -> bool:
        """对话语音：打断当前播放并立即播这条。"""
        if self._play_cmd is None:
            return False
        if not os.path.isfile(path):
            return False
        self._stop_current_proc()
        self._drain_queue()
        self._queue.put(("file", path, label or os.path.basename(path)))
        return True

    def announce_lan_ip(self, ip: str, *, force: bool = False) -> None:
        """等「蓝牙就绪待连接」播完后，阻塞播报中文 IP：每个数字 0.5 秒，连播 2 次，间隔 3 秒。"""
        ip = (ip or "").strip()
        if not ip or self._play_cmd is None:
            return
        key = f"lan_ip:{ip}"
        now = time.monotonic()
        last = self._last_play_ts.get(key, 0.0)
        if not force and now - last < IP_ANNOUNCE_COOLDOWN_SEC:
            return
        self._last_play_ts[key] = now
        threading.Thread(
            target=self._announce_ip_rounds,
            args=(ip,),
            daemon=True,
            name="ip-announce",
        ).start()

    def _play_clip_for(self, path: str, duration: float, env: dict) -> None:
        if not os.path.isfile(path) or self._play_cmd is None:
            time.sleep(duration)
            return
        proc: Optional[subprocess.Popen] = None
        try:
            proc = subprocess.Popen(
                [*self._play_cmd, path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=env,
            )
            with self._play_lock:
                self._current_proc = proc
            deadline = time.monotonic() + max(0.05, float(duration))
            while time.monotonic() < deadline:
                if proc.poll() is not None:
                    remain = deadline - time.monotonic()
                    if remain > 0:
                        time.sleep(remain)
                    break
                time.sleep(0.02)
            if proc.poll() is None:
                proc.kill()
                try:
                    proc.wait(timeout=0.3)
                except subprocess.TimeoutExpired:
                    pass
        except (OSError, subprocess.SubprocessError) as e:
            self._log(f"[voice] IP 片段播放失败: {e}")
            time.sleep(duration)
        finally:
            with self._play_lock:
                if proc is not None and self._current_proc is proc:
                    self._current_proc = None

    def _announce_ip_rounds(self, ip: str) -> None:
        if not self._ip_announce_lock.acquire(blocking=False):
            self._log("[voice] IP 播报进行中，跳过重复")
            return
        try:
            from ip_tts import (
                DIGIT_SEC,
                IP_GAP_SEC,
                IP_REPEAT,
                ensure_digit_clips,
                ip_clip_sequence,
            )
        except ImportError:
            self._log("[voice] 无法加载 ip_tts，跳过 IP 播报")
            return
        try:
            if not ensure_digit_clips():
                self._log(
                    f"[voice] 中文数字语音生成失败: {ip}（请确认已安装 espeak-ng）"
                )
                return
            clips = ip_clip_sequence(ip)
            if not clips:
                self._log(f"[voice] IP 无有效数字: {ip}")
                return
            if not self._ready_done.wait(timeout=25.0):
                self._log("[voice] 等待「蓝牙就绪待连接」超时，仍继续播 IP")
            env = _audio_env()
            self._log(
                f"[voice] 开始汉语 IP 播报 {ip}（每个数字 {DIGIT_SEC}s × {IP_REPEAT} 次，间隔 {IP_GAP_SEC}s）"
            )
            for n in range(IP_REPEAT):
                if self._stop.is_set():
                    return
                for path, dur in clips:
                    if self._stop.is_set():
                        return
                    self._play_clip_for(path, dur, env)
                if n < IP_REPEAT - 1:
                    time.sleep(IP_GAP_SEC)
            self._log(f"[voice] IP 播报完成: {ip}")
        finally:
            self._ip_announce_lock.release()

    def _play_wav_blocking(self, path: str) -> None:
        """阻塞播完整个 WAV，不打断、不限 0.5 秒切片。"""
        if self._play_cmd is None or not os.path.isfile(path):
            return
        env = _audio_env()
        proc: Optional[subprocess.Popen] = None
        try:
            proc = subprocess.Popen(
                [*self._play_cmd, path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=env,
            )
            with self._play_lock:
                self._current_proc = proc
            proc.wait(timeout=20.0)
        except subprocess.TimeoutExpired:
            if proc is not None:
                proc.kill()
                try:
                    proc.wait(timeout=0.3)
                except subprocess.TimeoutExpired:
                    pass
        except (OSError, subprocess.SubprocessError) as e:
            self._log(f"[voice] 阻塞播放失败: {e}")
        finally:
            with self._play_lock:
                if proc is not None and self._current_proc is proc:
                    self._current_proc = None

    def on_ble_ready(self) -> None:
        if self._ready_played:
            return
        self._ready_played = True
        threading.Thread(target=self._play_ble_ready_blocking, daemon=True).start()

    def _play_ble_ready_blocking(self) -> None:
        fname = PROMPTS["ble_ready"][0]
        path = os.path.join(self._assets_dir, fname)
        self._log("[voice] 播放: 蓝牙就绪，待连接")
        self._play_wav_blocking(path)
        self._ready_done.set()

    def on_ble_connected(self) -> None:
        self.play("ble_connected")

    def on_ble_disconnected(self) -> None:
        self._ready_played = False
        self._ready_done.clear()
        self.play("ble_disconnected")

    def on_wifi_changed(self) -> None:
        self.play("wifi_changed")

    def on_wifi_connected(self) -> None:
        self.play("wifi_connected")

    def on_wifi_disconnected(self) -> None:
        self.play("wifi_disconnected")

    def _worker(self) -> None:
        env = _audio_env()
        while not self._stop.is_set():
            try:
                item = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if item is None:
                break
            if isinstance(item, tuple) and item[0] == "unlink":
                try:
                    os.unlink(item[1])
                except OSError:
                    pass
                continue
            if isinstance(item, tuple) and item[0] == "file":
                _, path, label = item
            else:
                prompt_id = str(item)
                fname, label = PROMPTS.get(prompt_id, (None, prompt_id))
                if not fname:
                    continue
                path = os.path.join(self._assets_dir, fname)
            if self._play_cmd is None or not os.path.isfile(path):
                continue
            proc: Optional[subprocess.Popen] = None
            try:
                proc = subprocess.Popen(
                    [*self._play_cmd, path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    env=env,
                )
                with self._play_lock:
                    self._current_proc = proc
                proc.wait(timeout=60.0)
                self._log(f"[voice] 播放: {label}")
            except (OSError, subprocess.SubprocessError) as e:
                self._log(f"[voice] 播放失败 {label}: {e}")
            finally:
                with self._play_lock:
                    if proc is not None and self._current_proc is proc:
                        self._current_proc = None
