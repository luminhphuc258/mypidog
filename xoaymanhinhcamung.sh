#!/usr/bin/env bash
set -euo pipefail

chmod +x ./rotate_touch.sh

# Tạo autostart để mỗi lần login vào Desktop/X11 là tự chạy
mkdir -p "$HOME/.config/autostart"

cat > "$HOME/.config/autostart/rotate_touch.desktop" <<'EOF'
[Desktop Entry]
Type=Application
Name=Rotate Touch 180
Exec=/bin/bash -lc "$HOME/mypidog/rotate_touch.sh"
X-GNOME-Autostart-enabled=true
EOF

echo "[OK] Installed autostart: $HOME/.config/autostart/rotate_touch.desktop"
echo "Giờ bạn có thể test ngay bằng:"
echo "  ./rotate_touch.sh"
echo "Hoặc reboot để nó tự chạy khi login."
