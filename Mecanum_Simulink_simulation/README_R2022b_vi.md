# Project Simulink robot Mecanum: MBC vs PID (MATLAB R2022b)

## 1. File thực sự tạo ra

Bộ ZIP chứa **script sinh mô hình Simulink**. Tớ không có MATLAB/Simulink engine trong môi trường tạo file, nên **không thể khẳng định đã dựng và chạy một file `.slx` thật tại đây**. Khi chạy `build_mecanum_simulink_R2022b` trên MATLAB R2022b có Simulink của cậu, script sẽ tạo file `mecanum_control_benchmark.slx` với các block thật (không phải ảnh).

Thiết kế dựa trên controller_MBC.m, controller_PID.m, robot_dynamics.m, parameters.m mà cậu gửi; và trajectory.m mở rộng từ lượt trước.

## 2. Tạo .slx trên máy

1. Giải nén ZIP ra một thư mục, ví dụ `D:\\MATLAB\\MecanumSimulink`.
2. Mở MATLAB R2022b, chọn **Current Folder** là thư mục vừa giải nén.
3. Gõ:

```matlab
build_mecanum_simulink_R2022b
```

Script tạo `mecanum_control_benchmark.slx`, thiết lập solver fixed-step 0.01s và kiểm tra `Update Diagram`. Nếu bị lỗi, MATLAB sẽ hiện vị trí/lý do cụ thể để chỉnh.

4. Mô phỏng một trường hợp:

```matlab
T = run_simulink_benchmark('circle','nominal');
```

5. Chạy đủ 12 cấu hình (hai bộ điều khiển, tổng 24 lượt):

```matlab
all_results = run_all_simulink_benchmarks;
```

6. Chỉ thay quỹ đạo và điều kiện, rồi nhấn Run trong Simulink:

```matlab
configure_simulink_benchmark('eight','disturbance');
open_system('mecanum_control_benchmark');
```

## 3. Kiến trúc mô hình

```text
                            +---- MBC_Controller -> Sum(tau+d) -> Dynamics -> Euler(q,nu) ---+
Clock -> Trajectory Generator|                                                  ^               |
             qd, vd, ad     |                                                  |               |
                            +---- PID_Controller -> Sum(tau+d) -> Dynamics -> Euler(q,nu) ---+|
                                      ^                   ^                                    ||
                               Unit Delay (I)       Disturbance                              ||
                                      |                                                        ||
                                      +----- Integral update                                  ||
                                                                                 Scope / To Workspace
```

Mỗi nhánh có plant độc lập. Tín hiệu đầu vào chung là cùng quỹ đạo, tham số controller, tham số plant, và nhiễu.

## 4. Block thực sự dùng

- `Clock`, `Constant`
- `MATLAB Function` x 9 (trajectory, disturbance, MBC, PID, 2 dynamics, error, 2 wheel-speed monitor)
- `Discrete-Time Integrator` x4 (Euler forward)
- `Unit Delay` x1 (bộ nhớ sai số tích phân PID)
- `Sum` x2 (cộng nhiễu)
- `Subsystem` x2 (đóng vòng MBC và PID)
- `Demux`, `Mux`, `Scope` x7
- `To Workspace` x10 (timeseries)

**Không dùng `powergui`:** không có mô hình mạch điện của Simscape Electrical Specialized Power Systems. Thêm `powergui` không làm mô hình lực tổng quát hiện tại chính xác hơn.

## 5. Cấu hình thí nghiệm

- 4 quỹ đạo: `circle`, `ellipse`, `eight`, `lateral_sine`.
- 3 điều kiện: `nominal`, `payload_15kg`, `disturbance`.
- Thời gian: 0–40 s; bước: 0.01 s; Solver: `FixedStepDiscrete`.
- Quán tính danh định: m = 10 kg, Iz = 0.8 kg m².
- Payload: m = 15 kg, Iz tăng theo tỷ lệ, controller vẫn dựa trên 10 kg.
- Nhiễu: [15; -10; 2] trong 15–20 s.
- Sai lệch khởi tạo so với điểm đầu của mỗi quỹ đạo: [+0.2m;-0.1m;+10°].
- Controller sử dụng đúng công thức đã có trong bản MATLAB gốc; MBC feedforward hiện vẫn là dạng rút gọn.

## 6. Cách mở Scope

Double click `Scope_X_Tracking`, `Scope_Y_Tracking`, `Scope_Heading_Tracking`, `Scope_Position_Error`, `Scope_Heading_Error`, `Scope_MBC_Forces`, `Scope_PID_Forces` và nhấn Run.

**Scope X/Y/Heading:** ba tín hiệu theo thứ tự reference, MBC, PID. **Position/Heading Error:** hai tín hiệu theo thứ tự MBC, PID. Force Scope: ba kênh Fx [N], Fy [N], Mz [N.m], lưu ý không cùng đơn vị.

## 7. Nghiệm thu và metric

`run_simulink_benchmark` so sánh trạng thái đầu ra Simulink với MATLAB `simulate_case.m` và in `MaxDiffMBC`, `MaxDiffPID`; ngưỡng chẩn đoán đề xuất 1e-6. Xuất bảng metric trong `results_simulink/*.csv` (RMSE sau 5s, yaw, control effort, saturation, recovery) và ảnh quỹ đạo/error. 

**Ràng buộc khoa học:** chưa có motor điện, bộ biến đổi nghịch lưu, ma sát tiếp xúc roller, slip, hay giới hạn tốc độ từng bánh. Wheel Speed chỉ là quan sát từ J_m nu. Vì vậy chỉ kết luận về bộ điều khiển trên plant động lực học thân robot hiện tại.

## 8. Lưu ý kỹ thuật / khắc phục lỗi

- Nếu lệnh `build_mecanum_simulink_R2022b` báo không tìm thấy Simulink, cần đảm bảo Simulink Toolbox được cài và license hoạt động.
- Nếu `Stateflow.EMChart` không khởi tạo, đảm bảo có phần **MATLAB Function** của Simulink.
- Nếu đường dẫn chứa phiên bản tên `(3)` hay `(4)` từ file gốc, bộ ZIP này đã chuẩn hóa thành tên `controller_MBC.m`, `parameters.m` ...
- Nếu thay thông số trong `parameters.m`, chạy `configure_simulink_benchmark(...)` lại trước mỗi lần Run (và lưu .slx nếu muốn giữ cấu hình).
- Không mở `.slx` trong phiên bản MATLAB **cũ hơn** R2022b. Script này hướng đến R2022b.
- Một số tham số biểu đồ/scope có thể khác giữa các bản cài đặt; phản hồi đầy đủ thông báo lỗi MATLAB để sửa.
