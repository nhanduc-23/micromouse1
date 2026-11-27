# 🐭 Micromouse - Hệ thống Robot Tự hành Giải Mê cung

Dự án thiết kế, chế tạo và lập trình robot **Micromouse** tự hành có khả năng cảm nhận không gian, dựng bản đồ theo thời gian thực và tìm đường đi ngắn nhất trong mê cung chưa biết trước (Học phần *Nhập môn Cơ điện tử*).

---

## ✨ Tính năng chính

* **Nhận biết môi trường chính xác:** Sử dụng mảng 4 cảm biến khoảng cách Laser ToF (`VL53L0X`) bố trí ở các hướng Trái, Trước-Trái, Trước-Phải và Phải để phát hiện tường và căn chỉnh tâm hành lang.
* **Điều khiển chuyển động vòng kín (PID):** Kết hợp dữ liệu từ bộ đếm xung `Encoder` và vận tốc góc trục Z từ `IMU MPU-6500` (Sensor Fusion) để giữ thẳng, bám tường và quay góc vuông 90° / 180° chính xác.
* **Hai chế độ vận hành độc lập (FSM):**
  * **Chế độ 1 – Exploration (Dò đường):** Tự động khám phá mê cung (lưới tối đa 16×16 ô), cập nhật bản đồ đối xứng và tìm đường đến đích bằng thuật toán **Flood-fill**.
  * **Chế độ 2 – Speedrun (Chạy tốc độ cao):** Đọc dữ liệu bản đồ đã lưu trong bộ nhớ **Flash nội** của STM32 để chạy theo lộ trình tối ưu nhất với tốc độ cao.
* **Gỡ lỗi không dây (Telemetry):** Truyền thông số cảm biến, trạng thái và tọa độ về máy tính qua Bluetooth `JDY-31` (115200 bps).

---

## 🛠️ Phần cứng sử dụng (Hardware)

| Khối chức năng | Linh kiện | Thông số / Ghi chú |
| :--- | :--- | :--- |
| **Vi điều khiển** | STM32F411CEU6 (Black Pill) | ARM Cortex-M4, 40 chân, nạp qua ST-Link V2 |
| **Cảm biến khoảng cách** | 4 × VL53L0X V2 | Cảm biến Laser ToF, giao tiếp I2C1 (cấp địa chỉ riêng qua XSHUT) |
| **Cảm biến quán tính** | IMU MPU-6500 | 6-DOF (sử dụng Gyro-Z & Accel), đặt tại chính giữa tâm xoay xe |
| **Động cơ & Truyền động** | 2 × GA12-N20 (6V, 500 RPM) | Tích hợp Encoder từ tính 2 kênh A/B, bánh xe ZJ327 (Ø34mm) |
| **Mạch điều khiển động cơ** | TB6612FNG | Cầu H đôi, kèm tụ hóa 470µF chống sụt áp và tụ gốm 104 triệt nhiễu |
| **Nguồn & Ổn áp** | Pin Li-Po 2S 7.4V + MP1584 | Hạ áp trực tiếp từ 7.4V xuống **3.3V** nuôi toàn bộ hệ thống logic |
| **Giao tiếp không dây** | Module Bluetooth JDY-31 | SPP 3.0, giao tiếp UART |

---

## 📌 Sơ đồ phân bổ chân (Pinout - STM32F411CEU6)

| Phân hệ | Thiết bị / Tín hiệu | Chân STM32 | Chức năng / Ngoại vi |
| :--- | :--- | :--- | :--- |
| **Động cơ (TB6612FNG)** | PWMA / PWMB | `PA8` / `PA9` | `TIM1_CH1` / `TIM1_CH2` (PWM) |
| | AIN1, AIN2 / BIN1, BIN2 | `PB12, PB13` / `PB14, PB15` | `GPIO_Output` (Điều khiển chiều quay) |
| | STBY | `PA5` | `GPIO_Output` (Kích hoạt driver mức HIGH) |
| **Encoder** | Encoder Trái (A / B) | `PA0` / `PA1` | `TIM2_CH1` / `TIM2_CH2` (Encoder Mode) |
| | Encoder Phải (A / B) | `PA6` / `PA7` | `TIM3_CH1` / `TIM3_CH2` (Encoder Mode) |
| **Cảm biến Laser** | I2C1 (SCL / SDA) | `PB6` / `PB7` | Bus `I2C1` dùng chung cho 4 cảm biến |
| | XSHUT 1 → 4 | `PA3`, `PA4`, `PB0`, `PB1` | `GPIO_Output` (Gán địa chỉ I2C lúc khởi động) |
| **IMU (MPU-6500)** | I2C2 (SCL / SDA) | `PB10` / `PB3` | Bus `I2C2` độc lập tránh nghẽn bus |
| **Bluetooth (JDY-31)** | TX / RX | `PA11` / `PA12` | `UART` (Baudrate: 115200) |
| **Giao diện người dùng** | Nút chuyển chế độ | `PA2` | `GPIO_EXTI2` (Ngắt ngoài sườn xuống) |
| | Nút Reset cứng / LED | `NRST` / `PC13` | Reset hệ thống / LED báo trạng thái |

---

## 🏗️ Kiến trúc phần mềm (Software Architecture)

Chương trình nhúng được viết bằng ngôn ngữ **C** trên nền tảng **STM32 HAL**, tổ chức theo mô hình phân lớp với chu kỳ vòng lặp điều khiển chính cố định **10ms (100Hz)**:

```text
micromouse1/
├── micromouse1.ioc              # File cấu hình ngoại vi STM32CubeMX
├── platformio.ini               # Cấu hình build & flash cho PlatformIO
└── Core/
    ├── Inc/                     # Các file header (.h)
    └── Src/                     # Mã nguồn triển khai theo 5 tầng:
        ├── config.h             # Hằng số cơ khí, tham số PID và Pin Mapping
        ├── tb6612fng.c          # [Platform] Driver điều khiển xung & chiều động cơ
        ├── encoder.c            # [Platform] Đọc xung từ Timer Encoder Mode
        ├── bt_debug.c           # [Platform] Truyền nhận dữ liệu gỡ lỗi qua Bluetooth
        ├── vl53l0x.c            # [Sensing] Khởi tạo địa chỉ I2C1 & đọc 4 cảm biến Laser
        ├── mpu6500.c            # [Sensing] Đọc Gyro-Z qua I2C2 tính góc quay
        ├── sensor_fusion.c      # [Motion] Tổng hợp sai số lệch tâm tường và góc hướng
        ├── pid.c                # [Motion] Bộ điều khiển PID vòng kín
        ├── motion_controller.c  # [Motion] Điều khiển tiến 1 ô, quay 90° / 180°
        ├── maze_solver.c        # [Navigation] Thuật toán Flood-fill & cập nhật bản đồ 16x16
        ├── flash_storage.c      # [Navigation] Lưu/đọc ma trận bản đồ vào Flash nội
        └── main.c               # [Application] Khởi tạo hệ thống & máy trạng thái (FSM)
