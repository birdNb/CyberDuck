#!/bin/bash
set -euo pipefail
cd /home/aidlux/microDuckPro
git config user.name bird
git config user.email liyp9503@qq.com
echo 'aidlux ALL=(ALL) NOPASSWD: /bin/systemctl stop serial-getty@ttyMSM0.service, /bin/systemctl start serial-getty@ttyMSM0.service' | sudo tee /etc/sudoers.d/ttymsm0-servo >/dev/null
sudo chmod 440 /etc/sudoers.d/ttymsm0-servo
python3 -c 'import serial,yaml; print("deps ok", serial.__version__, yaml.__version__)'
cat /home/aidlux/microDuckPro/servo/servo_config.yaml
echo SETUP_OK
