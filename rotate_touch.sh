#!/bin/bash

# Tìm ID touch device (ưu tiên Waveshare / Touch)
TOUCH_ID=$(xinput list | grep -i -E "waveshare|touch|ilitek|goodix|d-wav" | grep -o "id=[0-9]*" | head -n1 | cut -d= -f2)

if [ -z "$TOUCH_ID" ]; then
    echo "❌ Không tìm thấy touch device"
    xinput list
    exit 1
fi

echo "✅ Touch device ID = $TOUCH_ID"

# Xoay touch 180 độ
xinput set-prop "$TOUCH_ID" "Coordinate Transformation Matrix" \
-1 0 1 \
0 -1 1 \
0 0 1

echo "✅ Touch rotated 180°"
