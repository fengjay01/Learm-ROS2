# 检测pyserial是否安装正确
import time
import serial
print(serial.__file__)
print(hasattr(serial, 'Serial'))


# 检测串口是否正常打开

try:
    # 替换为你的串口号
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    print("✅ 串口打开成功")

    # 可选：发送一条测试数据
    test_message = "HELLO STM32\n"
    ser.write(test_message.encode('utf-8'))
    print(f"📤 已发送测试数据: {test_message.strip()}")

    # 可选：尝试读取回显（如有）
    time.sleep(0.5)
    if ser.in_waiting:
        response = ser.readline().decode('utf-8', errors='ignore')
        print(f"📥 收到回显: {response.strip()}")
    else:
        print("⚠️ 未收到回显（可正常）")

    ser.close()
    print("🔌 串口关闭")

except Exception as e:
    print(f"❌ 串口打开失败: {e}")
