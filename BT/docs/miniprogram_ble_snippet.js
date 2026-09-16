/**
 * microDuck BLE — 连接 / IP / 音量
 * FFE0/FFE1/FFE2 | 订阅 FFE2 后收 IP / pwr，无模式握手
 */
const SERVICE_UUID = '0000FFE0-0000-1000-8000-00805F9B34FB'
const WRITE_UUID = '0000FFE1-0000-1000-8000-00805F9B34FB'
const NOTIFY_UUID = '0000FFE2-0000-1000-8000-00805F9B34FB'
const TARGET_NAME = 'HT_88888888'
const CMD_COOLDOWN_MS = 800
const HEARTBEAT_MS = 2000

function normUuid(u) {
  return (u || '').replace(/-/g, '').toLowerCase()
}

function uuidHit(u, needle) {
  const n = normUuid(u)
  return n === needle || n.includes(needle)
}

function ab2str(buffer) {
  const arr = new Uint8Array(buffer)
  let s = ''
  for (let i = 0; i < arr.length; i++) s += String.fromCharCode(arr[i])
  try { return decodeURIComponent(escape(s)) } catch (e) { return s }
}

function str2ab(text) {
  const buf = new ArrayBuffer(text.length)
  const v = new Uint8Array(buf)
  for (let i = 0; i < text.length; i++) v[i] = text.charCodeAt(i)
  return buf
}

function delay(ms) {
  return new Promise((r) => setTimeout(r, ms))
}

function isIOS() {
  try { return wx.getSystemInfoSync().platform === 'ios' } catch (e) { return false }
}

Page({
  data: { deviceId: '', gattReady: false, robotIp: '', battery: -1 },

  _serviceId: '',
  _writeCharId: '',
  _notifyCharId: '',
  _writeNoResponse: false,
  _lastCmdAt: 0,
  _hbTimer: null,

  onUnload() {
    this.stopHeartbeat()
    if (this.data.deviceId) wx.closeBLEConnection({ deviceId: this.data.deviceId })
  },

  openAdapter() {
    wx.openBluetoothAdapter({
      mode: 'central',
      success: () => this.startScan(),
    })
  },

  startScan() {
    const opts = {
      allowDuplicatesKey: true,
      powerLevel: 'high',
      success: () => {
        wx.onBluetoothDeviceFound((res) => {
          res.devices.forEach((d) => {
            if (this.data.deviceId) return
            const name = (d.name || d.localName || '').toLowerCase()
            const uuids = d.advertisServiceUUIDs || []
            const serviceHit = uuids.some((u) => uuidHit(u, 'ffe0'))
            const nameHit = name.includes('ht_') || name === TARGET_NAME.toLowerCase()
            if (!nameHit && !serviceHit) return
            this.setData({ deviceId: d.deviceId })
            wx.stopBluetoothDevicesDiscovery({})
            this.establishBleLink(d.deviceId)
          })
        })
      },
    }
    wx.startBluetoothDevicesDiscovery(opts)
  },

  async establishBleLink(deviceId) {
    await this._createConnection(deviceId)
    await delay(isIOS() ? 2000 : 800)
    if (wx.setBLEMTU) wx.setBLEMTU({ deviceId, mtu: 247 })
    await this._discoverGatt(deviceId)
    await this._subscribeNotify(deviceId)
    await delay(isIOS() ? 350 : 150)
    this.setData({ gattReady: true })
    this.startHeartbeat()
  },

  _createConnection(deviceId) {
    return new Promise((ok, fail) => {
      wx.createBLEConnection({ deviceId, timeout: 10000, success: () => ok(), fail: fail })
    })
  },

  _discoverGatt(deviceId) {
    return new Promise((ok, fail) => {
      wx.getBLEDeviceServices({
        deviceId,
        success: (r) => {
          const s = r.services.find((x) => uuidHit(x.uuid, 'ffe0'))
          if (!s) return fail(new Error('no FFE0'))
          this._serviceId = s.uuid
          wx.getBLEDeviceCharacteristics({
            deviceId,
            serviceId: this._serviceId,
            success: (c) => {
              const w = c.characteristics.find((x) => uuidHit(x.uuid, 'ffe1'))
              const n = c.characteristics.find((x) => uuidHit(x.uuid, 'ffe2'))
              if (!w) return fail(new Error('no FFE1'))
              this._writeCharId = w.uuid
              this._notifyCharId = n ? n.uuid : ''
              const p = w.properties || {}
              this._writeNoResponse = !!(p.writeNoResponse || p.writeWithoutResponse)
              ok()
            },
            fail,
          })
        },
        fail,
      })
    })
  },

  _subscribeNotify(deviceId) {
    if (!this._notifyCharId) return Promise.resolve()
    wx.onBLECharacteristicValueChange((ev) => this.onNotify(ab2str(ev.value)))
    return new Promise((ok, fail) => {
      wx.notifyBLECharacteristicValueChange({
        deviceId,
        serviceId: this._serviceId,
        characteristicId: this._notifyCharId,
        state: true,
        success: ok,
        fail,
      })
    })
  },

  _write(text, requireResponse) {
    const deviceId = this.data.deviceId
    return new Promise((ok, fail) => {
      const req = {
        deviceId,
        serviceId: this._serviceId,
        characteristicId: this._writeCharId,
        value: str2ab(text),
        success: ok,
        fail,
      }
      if (!requireResponse && this._writeNoResponse) req.writeType = 'writeNoResponse'
      wx.writeBLECharacteristicValue(req)
    })
  },

  startHeartbeat() {
    this.stopHeartbeat()
    this._hbTimer = setInterval(() => {
      this._write('1', false).catch(() => {})
    }, HEARTBEAT_MS)
  },

  stopHeartbeat() {
    if (this._hbTimer) clearInterval(this._hbTimer)
    this._hbTimer = null
  },

  onNotify(text) {
    const line = (text || '').trim()
    const low = line.toLowerCase()
    if (line.startsWith('ACK:')) {
      console.log('ACK', line.slice(4))
      return
    }
    if (low.startsWith('ip:')) {
      this.setData({ robotIp: line.slice(3).trim() })
      return
    }
    if (low.startsWith('pwr:')) {
      this.setData({ battery: parseInt(line.slice(4), 10) })
      return
    }
    console.log('notify', line)
  },

  async sendCommand(text) {
    if (Date.now() - this._lastCmdAt < CMD_COOLDOWN_MS) return
    this._lastCmdAt = Date.now()
    await this._write(text, true)
  },

  async sendVolume(percent) {
    const pct = Math.max(0, Math.min(100, Math.round(Number(percent) || 0)))
    await this.sendCommand(`V ${pct}`)
  },

  sendRename(digits8) {
    const d = String(digits8 || '').replace(/\D/g, '').slice(-8).padStart(8, '0')
    return this.sendCommand(`rename HT_${d}`)
  },

  sendWifi(ssid, password) {
    return this.sendCommand(`WIFI ${ssid} ${password}`)
  },

  /** 对话语音：录音文案前5字拼音首字母大写，如 BBMWX */
  sendConversation(code) {
    return this.sendCommand(String(code || '').trim().toUpperCase())
  },
})
