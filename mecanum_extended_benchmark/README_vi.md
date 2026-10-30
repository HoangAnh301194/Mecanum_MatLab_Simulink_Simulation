# MATLAB: benchmark Mecanum MBC vs PID — 4 quỹ đạo x 3 điều kiện

**Chạy:** đặt MATLAB Current Folder vào thư mục này, gõ `main_extended`.

Bộ file giữ nguyên thuật toán `controller_MBC.m`, `controller_PID.m`, `robot_dynamics.m`, `mecanum_kinematics.m`, `parameters.m` của người dùng. Các tệp được đổi tên đúng tên hàm MATLAB. `simulate_case.m` chỉ bổ sung dữ liệu log `nu`, đạo hàm quỹ đạo và điền lệnh điều khiển tại mẫu cuối để tính chi phí điều khiển, không thay đổi các bước cập nhật trạng thái.

**Các quỹ đạo (`trajectory.m`):**
- `circle` tròn (nguyên gốc: R = 2 m, w = 0.2 rad/s; đầu robot hướng tiếp tuyến)
- `ellipse` ellipse (a = 2 m, b = 1.2 m; hướng tiếp tuyến)
- `eight` hình số 8 (a = 2 m, b = 1.4 m; hướng tiếp tuyến)
- `lateral_sine` x = 0.35t m, y = 0.6sin(0.4t) m; thân robot giữ góc bằng 0

Đạo hàm vận tốc, gia tốc và góc hướng được tính **giải tích**, không dùng lấy sai phân nhiễu.

**Thiết kế thí nghiệm:**
- Cùng gain cho mọi quỹ đạo và điều kiện, controller/plant dùng tham số 10 kg danh định.
- Payload: plant 15 kg, Iz tăng theo tỉ lệ; controller vẫn model 10 kg.
- Nhiễu [15; -10; 2] (N, N, N*m) từ 15 đến 20 giây.
- Cùng sai lệch ban đầu `[+0.20 m; -0.10 m; +10 độ]` so với trạng thái tham chiếu tại t=0, cho tất cả quỹ đạo và controller. Đây là cấu hình **mới** nhằm so sánh ngang nhau. Nó không trùng kết quả CSV gốc vì CSV gốc khởi tạo `[0;0;0]`.
- Thời gian 40 giây, bước Euler 0.01 giây, hai controller MBC/PID.

**Metrics (`compute_metrics.m`):**
- `RMSE_all_m`: RMSE vị trí từ t=0.
- `RMSE_first5_m`, `RMSE_after5_m`, `RMSE_after10_m`: chia giai đoạn 0–5, 5–40, 10–40 giây để thấy ảnh hưởng lựa chọn thời gian quá độ; RMSE 5–40 dùng làm chỉ số chính.
- `RMSE_along_after5_m`, `RMSE_cross_after5_m`: chiếu sai số vị trí theo tiếp tuyến/pháp tuyến của hướng chuyển động tham chiếu tại cùng thời điểm (không phải khoảng cách ngắn nhất đến đường cong).
- `RMSE_yaw_all_deg`, `RMSE_yaw_after5_deg`: RMSE góc heading tính bằng độ, đã wrap về [-pi,pi].
- `MAE_after5_m`, `P95_after5_m`, `Max_after5_m`, `IAE_after5_m_s`.
- `Control_effort_norm`: trung bình thời gian của sum[(Fx/100)^2+(Fy/100)^2+(Mz/50)^2]. Thấp hơn là dùng lực/moment điều khiển ít hơn theo chuẩn này. Không đồng nghĩa tiêu thụ điện thấp hơn.
- `Saturation_pct`: tỉ lệ mẫu thời gian mà ít nhất 1 trong 3 lực/mô-men đạt >=99.9% giới hạn.
- `Control_variation_per_s`: biến thiên tuyệt đối tích lũy của tín hiệu đã chuẩn hóa trên 1 giây.
- `Peak_wheel_rad_s`: tốc độ bánh lớn nhất; chỉ là chẩn đoán, plant chưa giới hạn actuator/bánh.
- `Mechanical_work_proxy_J`: tích phân tổng trị tuyệt đối Fx*vx + Fy*vy + Mz*omega theo từng kênh; đây là proxy cơ học, **không phải năng lượng điện**, không tính tổn hao motor.
- `Recovery_s`: chỉ TN3, thời gian tính từ sau t=20 đến khi e_position<=0.10m VÀ |e_yaw|<=5 độ liên tục 2 giây. NaN = chưa đạt điều kiện trong cửa sổ mô phỏng.
- `Delta_RMSE_during_dist_m`, `Excess_error_area_15_30_m_s` so với run danh định ghép cặp đúng quỹ đạo/controller, đo tác động tăng thêm do nhiễu, thay vì quy kết sai số bám danh định là sai số do nhiễu.

**Đầu ra:** `results_extended/all_metrics.csv` (24 hàng), `paired_comparison.csv` (12 hàng), `disturbance_impact.csv` (8 hàng); mỗi quỹ đạo có ảnh `path_*.png` và `error_*.png`.

**Giới hạn khoa học bắt buộc nêu khi báo cáo:**
1. Tín hiệu `tau` là lực tổng quát trực tiếp trong mô hình thân robot; chưa mô hình ánh xạ motor torque -> lực thân, ma sát tiếp xúc/roller, hoặc giới hạn tốc độ bánh trong plant. Chưa có trượt.
2. Model-based controller gốc sử dụng `R' * qd_ddot` làm feedforward, bỏ qua đạo hàm của `R'` và các thành phần đạo hàm vận tốc tham chiếu, vì thế không phải inverse-dynamics tracking hoàn chỉnh; nên ghi là `MBC (existing simplified implementation)`.
3. Không điều chỉnh gain lại theo từng quỹ đạo. Để xuất bản nghiêm túc, nên bổ sung nghiên cứu độ nhạy gain và mô phỏng sai lệch thông số/nhiễu ngẫu nhiên nhiều lần.
4. Các quỹ đạo có độ dài và phổ tốc độ khác nhau; không xếp hạng controller bằng cách so giá trị RMSE của các quỹ đạo khác nhau. Chỉ so MBC–PID trong cùng một hàng điều kiện, dùng hiệu năng tổng thể (vị trí + góc + effort + saturation).
5. Nếu cần xét sức tải phần cứng, phải bổ sung actuator saturation đúng với giới hạn torque/speed của động cơ và bánh; tính `Peak_wheel_rad_s` không tự áp giới hạn.

**Chú ý:** `main_original.m` là bản sao lưu nguyên mã `main(4).m`, dùng `main_extended.m` để chạy loạt mô phỏng mới.
