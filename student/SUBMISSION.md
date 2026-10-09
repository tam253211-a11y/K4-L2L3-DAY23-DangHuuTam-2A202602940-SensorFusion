# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Đặng Hữu Tâm
- MSSV: 2A202602940
- Email:
- Link repo (fork): https://github.com/tam253211-a11y/K4-L2L3-DAY23-DangHuuTam-2A202602940-SensorFusion
- Commit hash nộp (`git rev-parse HEAD`): commit `CP6` cuối cùng trên `main`, hash 40 ký tự nộp kèm trên LMS

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: `compare`, `[0, 198]` (199 frame),
  `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`, `seed = 0`
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: precision 0.9701, recall 0.7004,
  tp/fp/fn = 519 / 16 / 222
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`:
  0.1503 m, 502, 11.344 m², 0, 239, 2.523
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`:
  0.1359 m, 502, 9.267 m², 0, 239, 2.523
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss:

| Chỉ số | LiDAR | Fused |
|---|---|---|
| RMSE (m) | 0.1503 | 0.1359 |
| matches | 502 | 502 |
| ghost_track_frames | 0 | 0 |
| missed_gt_frames | 239 | 239 |
| precision_track = matches/(matches+ghost) | 1.000 | 1.000 |
| coverage = matches/det_tp = 502/519 | 0.967 | 0.967 |

  - **Hai mode có cùng matches, ghost, miss và `mean_confirmed_tracks`.** Đây là
    kết quả mong đợi của track-then-fuse: camera không tạo, không xóa và không
    đổi score track (`manager.py` chỉ chạy lifecycle khi `sensor.name == "lidar"`),
    nên tập confirmed tracks ở mỗi frame giống hệt nhau. Vì vậy so sánh RMSE giữa
    hai mode là công bằng (cùng 502 cặp), không phải RMSE thấp nhờ ghép ít hơn.
  - **Khác biệt nằm ở độ chính xác vị trí.** `sum_sq_err` giảm từ 11.344 xuống
    9.267 m² (≈ −18%), RMSE giảm 0.0145 m. So từng frame trong
    `grade_run_lidar.log`/`grade_run_fused.log`: trong 195 frame có cặp ghép,
    fused có `sum_sq_err` nhỏ hơn ở 136 frame. Camera cho thêm ràng buộc góc
    (u ↔ y/x, v ↔ z/x) nên EKF update kéo vị trí theo hướng ngang/đứng.
  - **Ghost = 0** vì một track phải có 4 hit LiDAR liên tiếp sau khi khởi tạo
    (score 1/6 → 5/6 > 0.8) mới được confirm (track đầu tiên confirm ở frame 4 trong
    log), còn track chưa confirm bị xóa ngay ở miss đầu tiên (score về 0). 16 detection
    FP vì thế không bao giờ thành confirmed track.
  - **239 miss** chủ yếu do detector: 222 xe hợp lệ không được phát hiện (`det_fn`),
    tracker không thể ghép xe không có đo. Phần còn lại là độ trễ khởi tạo (frame 0–3
    chưa có confirmed track, mỗi frame miss 2 xe). Do đó coverage trên `det_tp` là 0.967.
  - **Giới hạn:** đo camera là tâm hộp 2D ground-truth FRONT cộng nhiễu seeded
    (σ = 5 px), không phải detector ảnh; không có false positive/miss camera, không lệch
    calibration. Mức cải thiện 0.0145 m vì thế là cận lạc quan và không chứng minh chất
    lượng một camera detector thật.

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. Khác biệt đo lidar 3D và camera 2D trong EKF (`z`, `R`)?

   **LiDAR:** `z = (x, y, z)ᵀ` (3×1, mét) là tâm hộp 3D; `h(x)` tuyến tính
   (`R_rot·p + t`), `H` = 3×6 hằng (khối xoay, cột vận tốc = 0);
   `R = diag(0.1², 0.1², 0.1²)` m². **Camera:** `z = (u, v)ᵀ` (2×1, pixel);
   `h(x)` phi tuyến: `u = c_i − f_i·y_s/x_s`, `v = c_j − f_j·z_s/x_s`
   (`camera_fusion.py`, `camera_measurement_prediction`), nên `H` là Jacobian 2×6
   phụ thuộc trạng thái (platform tính bằng chain rule); `R = diag(5², 5²)` px²
   (`build_camera_measurement`). Camera không đo độ sâu nên chỉ ràng buộc hướng nhìn,
   còn LiDAR ràng buộc cả vị trí 3D. Cả hai đi qua cùng `ekf_update` vì hàm chỉ gọi
   `meas.sensor.get_H/get_hx` và `meas.R` (`kalman.py`).

2. Vì sao cần gating Mahalanobis trước khi gán?

   `d² = γᵀS⁻¹γ` với `S = HPHᵀ + R` (`association.py`, `mahalanobis_distance`)
   chuẩn hóa sai lệch theo độ bất định của track và sensor: track mới (σ vận tốc
   50 m/s) có cổng rộng, track ổn định có cổng hẹp; d² không đơn vị nên dùng chung cho
   mét (LiDAR) và pixel (camera), điều Euclid không làm được. Nếu cặp đúng, d² ~ χ²
   với bậc tự do = số chiều đo, nên cổng `chi2.ppf(0.995, dim)` (≈ 12.84 cho LiDAR,
   ≈ 10.60 cho camera) loại các cặp gần như chắc chắn sai trước khi greedy chọn cặp
   rẻ nhất. Không gating thì một detection FP xa vẫn có thể bị gán vào track khi
   không còn ứng viên nào, làm lệch state. Cặp ngoài FOV được gán `inf` trước khi
   tính d² (`association_cost_matrix`) để không chiếu điểm sau camera.

3. Pipeline là track-then-fuse hay fuse-then-track? Chỉ ra trên log `fusion-run-lab`.

   **Track-then-fuse.** Mỗi frame trong `platform/fusion_lab/scripts/run_lab.py`:
   `KF.predict(track)` một lần cho mọi track (dòng 200) → `associate_and_update(...,
   lidar_sensor)` (dòng 202) → nếu fused, `associate_and_update(..., camera_sensor)`
   (dòng 209) trên **cùng** danh sách track. Không có bước gộp raw LiDAR + ảnh trước
   detection. Trên log: `grade_run_lidar.log` và `grade_run_fused.log` có
   `det_tp/det_fp/det_fn`, `confirmed`, `matches`, `ghosts`, `misses` **trùng nhau ở
   cả 199 frame** (cùng detector, cùng lifecycle LiDAR); chỉ `sum_sq_err` khác
   (ví dụ tổng 11.344 → 9.267 m²). Điều đó cho thấy camera chỉ tham gia sau khi track
   đã tồn tại, như một lần EKF update bổ sung.

4. Nếu camera lệch calibration, triệu chứng gì trên innovation/residual?

   Extrinsic sai làm `h(x)` dự đoán pixel lệch một lượng **có hệ thống**: innovation
   camera `γ = z − h(x)` không còn trung bình 0 mà có bias ổn định (cùng dấu, cùng
   hướng) qua nhiều frame và nhiều track, và d² trung bình vượt kỳ vọng χ²₂ (≈ 2).
   Lệch nhỏ: cặp vẫn qua cổng, EKF bị kéo về sai vị trí → residual LiDAR ở frame sau
   tăng ngược chiều (LiDAR và camera "giằng co"), RMSE fused tăng, có thể vượt LiDAR.
   Lệch lớn: d² vượt ngưỡng ≈ 10.6, cặp camera bị gate loại, fused thoái hóa về
   LiDAR-only. Lệch góc (rotation) δθ gây bias pixel gần như không đổi (≈ f·δθ, với
   f ≈ 2000 px thì 0.1° ≈ 3.5 px), nhưng quy ra mét thì sai số tăng theo khoảng cách;
   lệch tịnh tiến t gây bias pixel ≈ f·t/độ sâu, nên lớn ở xe gần và nhỏ ở xe xa.

5. Vì sao `associate_and_update(..., sensor)` cần sensor tường minh ở frame rỗng?
   Giải thích vì sao lidar quyết định score/init/delete còn camera chỉ EKF update.

   Khi `meas_list` rỗng không thể suy ra sensor từ `meas.sensor`, nhưng
   `manage_tracks(unassigned_tracks, [], sensor)` vẫn phải chạy: với LiDAR, frame rỗng
   nghĩa là mọi track trong FOV bị **miss** → trừ `1/window`, rồi xét xóa; nếu bỏ qua
   thì ghost sống mãi. Với camera, cùng lời gọi phải **không** làm gì với lifecycle
   (`manager.py`: `if sensor.name != "lidar": return`). Code: `associate_and_update`
   luôn kết thúc bằng `manager.manage_tracks(..., sensor)` kể cả khi không có cặp.
   LiDAR quyết định tồn tại vì cho vị trí 3D đầy đủ, FOV rộng (±90° quanh hướng
   trước trong lab) và là nguồn tạo detection. Camera FRONT chỉ thấy khoảng ±24.7°
   (tính từ intrinsics và bề rộng ảnh của segment), không có độ sâu, và trong lab là
   đo mô phỏng từ nhãn; nếu camera đổi score, track ngoài FOV camera bị "miss" oan,
   track trong FOV được cộng hai lần mỗi frame, và một đo 2D không thể khởi tạo
   state 3D.

6. Nêu điều kiện xác nhận, giữ confirmed sau miss, và điều kiện xóa track.

   (`track_management.py`, `window = 6`.) Khởi tạo: score `1/6`, state
   `initialized`, P vị trí = R LiDAR xoay sang hệ xe, P vận tốc `diag(50², 50², 5²)`.
   LiDAR hit: `score = min(1, score + 1/6)`, track chưa confirmed thành `tentative`;
   miss trong FOV: `score −= 1/6`. **Xác nhận** khi `score > 0.8` (cần 4 hit sau khởi
   tạo; score đúng bằng 0.8 vẫn tentative). **Đã confirmed thì giữ confirmed** khi
   miss — score giảm (1 → 5/6) nhưng state không hạ. **Xóa** (OR): `P[0,0]` hoặc
   `P[1,1] > max_P = 9` m² (σ > 3 m); confirmed có `score < 0.6` (3 miss liên tiếp từ
   1.0); chưa confirmed có `score ≤ 0` (track mới miss 1 lần). Chỉ lượt LiDAR gọi
   các hàm này.

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- Không

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Claude (Claude Code, trong VS Code)
- Dùng cho phần nào (hàm, câu hỏi, debug): cài đặt môi trường và sửa lỗi numpy/MKL
  trên Windows (đổi sang OpenBLAS); viết code Part E (`kalman.py`), Part G
  (`camera_fusion.py`), Part F (`association.py`), Part H (`track_management.py`);
  chạy `fusion-run-lab --fusion compare --seed 0`, kiểm tra invariant log; soạn
  bản nháp báo cáo này và 6 câu giải thích; em yêu cầu AI giải thích từng hàm sau mỗi
  checkpoint.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): `pytest student/tests -q`
  (128 passed, không failed/xfailed); đối chiếu công thức EKF/Mahalanobis/pinhole
  với test số (ví dụ K = 0.5 trong test update, d² = 2 trong test Mahalanobis);
  chạy Waymo 199 frame, tính lại tổng từ `grade_run.log` khớp `metrics.json`;
  số liệu trong báo cáo lấy trực tiếp từ `student/artifacts/`;
  `python tools/check_submission.py`.

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [x] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [ ] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
