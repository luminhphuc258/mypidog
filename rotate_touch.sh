#!/usr/bin/env bash
set -euo pipefail

# --- cấu hình ---
# tên device touch trong `xinput list`
TOUCH_NAME="${TOUCH_NAME:-Waveshare WS170120}"

# Ma trận cho xoay 180 độ (inverted)
MATRIX=(-1 0 1  0 -1 1  0 0 1)

# --- đảm bảo chạy trong X11 session ---
: "${DISPLAY:=:0}"
export DISPLAY

# nếu chạy từ autostart / service mà thiếu XAUTHORITY
if [[ -z "${XAUTHORITY:-}" ]]; then
  export XAUTHORITY="$HOME/.Xauthority"
fi

# --- tìm id của touch ---
TOUCH_ID="$(xinput list --id-only "$TOUCH_NAME" 2>/dev/null || true)"

if [[ -z "$TOUCH_ID" ]]; then
  echo "[ERR] Không tìm thấy touch device tên: '$TOUCH_NAME'"
  echo "      Hãy chạy: xinput list  (xem đúng tên rồi sửa TOUCH_NAME)"
  exit 1
fi

echo "[OK] Touch device: '$TOUCH_NAME' id=$TOUCH_ID"
echo "[DO] Apply Coordinate Transformation Matrix (rotate 180)..."

xinput set-prop "$TOUCH_ID" "Coordinate Transformation Matrix" "${MATRIX[@]}"

echo "[DONE] Touch rotated 180°."
