// AGV SHIZAI 03/10/2026 
#include "control_step.h"
#include "sensor.h"

const int pin_bao_mat_line = 45;
const int pin_dung = 46; // Vẫn giữ cấu hình chân dự phòng
const int pin_tin_hieu_RF = 44;
const int pin_tin_hieu_11 = 11;
const int pin_tin_hieu_12 = 12;

unsigned long t_bat_dau_pin_11 = 0;
unsigned long t_bat_dau_pin_12 = 0;
bool dang_bat_pin_11 = false;
bool dang_bat_pin_12 = false;
const int toc_do_lui_15 = 6000;

// --- BIẾN KHIÊN MIỄN NHIỄM VẬT CẢN LÚC XẢ HÀNG ---
bool dang_mien_nhiem_an_toan = false;
unsigned long t_bat_dau_mien_nhiem = 0;

// --- BIẾN QUẢN LÝ CHẠY MÙ ---
const unsigned long time_chay_mu_sau_lui = 23000; // 23 giây chạy mù
unsigned long t_ket_thuc_lui = 0;
bool dang_chay_mu_sau_lui = false;
unsigned long t_bat_dau_lui_15 = 0;

// --- BIẾN QUẢN LÝ XI LANH (THAY THẾ NAM CHÂM) ---
const unsigned long time_dung_12 = 3000; // Khoảng thời gian từ lúc mở đến lúc đóng xi lanh
bool buoc_11_done = false; // Cờ theo dõi đã mở xi lanh chưa
bool buoc_12_done = false; // Cờ theo dõi đã đóng xi lanh chưa
// Thêm 2 biến này vào phần đầu code
bool dang_cho_dong_xi_lanh_khi_chay = false;
unsigned long t_bat_dau_cho_dong = 0;

const unsigned long time_bao_mat_line_lau = 10000;
const unsigned long thoi_gian_bo_qua_mat_line = 500;
const unsigned long time_hu_line = 50;
const int time_tang_toc_case14_lan_1 = 8000; // 8s chờ tín hiệu
const unsigned long time_dung_lay_hang = 20000; // 20s dừng
const int speed = 1220;
const int slowSpeed = 12000;
const int startSpeed = 12000;

const int BU_ZONE_123 = 0;
const int BU_ZONE_456 = 400;
const int BU_ZONE_789 = 500;
const int tg_gui_tinh_hieu_rf = 3000;

const unsigned long thoi_gian_tang_toc = 9000;
const unsigned long thoi_gian_giam_toc = 6000;
const unsigned long THOI_GIAN_QUAY_MU = 4000;

const float Kp_123 = 50.0; const float Kd_123 = 500.0;
const float Kp_456 = 150.0; const float Kd_456 = 500.0;
const float Kp_789 = 200.0; const float Kd_789 = 500.0;
const float Ki = 0.01;
const float TIME_VAO_CUA = 200.0;
const float TIME_RA_CUA  = 300.0;

enum TrangThaiTram { 
    CHAY_BINH_THUONG=0, BAT_DAU_GIAM_TOC=1, CHO_DUNG=2, 
    DANG_DUNG_LAY_HANG=3, ROI_KHOI_TRAM=4, QUAY_DAU=5, 
    TAM_DUNG_DE_THA=6, LUI_CASE_15=7, TAM_DUNG_SAU_LUI=8
};
TrangThaiTram trang_thai_14 = CHAY_BINH_THUONG;
unsigned long t_bat_dau_dung_sau_lui = 0; 

unsigned long thoi_gian_cap_nguon = 0, thoi_gian_bat_dau = 0, thoi_gian_bat_dau_giam = 0, t_dung = 0;
unsigned long t_chong_doi_tram = 0, last_ramp_time = 0, thoi_gian_bat_dau_quay = 0;
unsigned long thoi_gian_mat_line = 0, t_bat_dau_tam_dung = 0, thoi_gian_roi_tram = 0;
bool dang_mat_line = false, dang_dung_vi_vat_can = false, dang_dung_vi_mat_line = false;
bool dang_tang_toc = true, dang_giam_toc = false, is_special_code = false;
int speed_quay_dau = 4000, raw_sensor = 0, steering_error = 0, last_valid_sensor = 0;
float toc_do_khi_bat_dau_giam = 0, toc_do_khi_bat_dau_tang = 0, currentSpeedVal = startSpeed, smoothBaseSpeed = 0;
float smoothedError = 0, lastError = 0, integral = 0, lastPidOutput = 0;

void setup() {
    pinMode(pin_dung, OUTPUT); digitalWrite(pin_dung, HIGH);
    pinMode(pin_bao_mat_line, OUTPUT); digitalWrite(pin_bao_mat_line, HIGH);
    pinMode(pin_tin_hieu_RF, OUTPUT); digitalWrite(pin_tin_hieu_RF, HIGH);
    
    // --- KHỞI TẠO CHÂN 11 & 12 MẶC ĐỊNH LÀ HIGH ---
    pinMode(pin_tin_hieu_11, OUTPUT); digitalWrite(pin_tin_hieu_11, HIGH);
    pinMode(pin_tin_hieu_12, OUTPUT); digitalWrite(pin_tin_hieu_12, HIGH);
    
    pinMode(PUL_TRAI, OUTPUT); pinMode(DIR_TRAI, OUTPUT); pinMode(ENA_TRAI, OUTPUT);
    pinMode(PUL_PHAI, OUTPUT); pinMode(DIR_PHAI, OUTPUT); pinMode(ENA_PHAI, OUTPUT);
    pinMode(T4, INPUT); pinMode(T3, INPUT); pinMode(T2, INPUT); pinMode(T1, INPUT);
    pinMode(TG, INPUT); pinMode(PG, INPUT);
    pinMode(P1, INPUT); pinMode(P2, INPUT); pinMode(P3, INPUT); pinMode(P4, INPUT);
    pinMode(cb1, INPUT_PULLUP);
    
    digitalWrite(ENA_TRAI, LOW); digitalWrite(ENA_PHAI, LOW);
    thoi_gian_cap_nguon = millis();
    KhoiTaoThongSoDeBa();
    if (In_SenSor() == 14 || In_SenSor() == 15) trang_thai_14 = ROI_KHOI_TRAM;
}

void loop() {
    raw_sensor = In_SenSor();
    
    // Tự động ngắt xi lanh 11, 12 đúng 1s (Chạy ngầm liên tục)
    XuLyCacTinHieuPhu(); 

    // Tự động tắt khiên miễn nhiễm sau 2 giây (2000ms) thả hàng
    if (dang_mien_nhiem_an_toan && (millis() - t_bat_dau_mien_nhiem >= 2000)) {
        dang_mien_nhiem_an_toan = false;
    }

    // CHỈ KIỂM TRA VẬT CẢN KHI KHÔNG CÓ KHIÊN MIỄN NHIỄM
    if (!dang_mien_nhiem_an_toan) {
        if (KiemTraAnToanGap() || XuLyMatLine()) return;
    }
    
    DocVaLocCamBien();
    XuLyMayTrangThai();
    QuyetDinhXuatXung();
}
void KhoiTaoThongSoDeBa() {
    currentSpeedVal = smoothBaseSpeed = toc_do_khi_bat_dau_tang = (float)startSpeed;
    thoi_gian_bat_dau = last_ramp_time = thoi_gian_roi_tram = millis();
    dang_tang_toc = true; dang_giam_toc = false;
    integral = lastError = smoothedError = lastPidOutput = 0;
}

bool KiemTraAnToanGap() {
    if (digitalRead(cb1) == LOW) {
        step_dc(false, false, HIGH, LOW, 0, 0);
        dang_dung_vi_vat_can = true;
        return true;
    } else if (dang_dung_vi_vat_can) {
        KhoiTaoThongSoDeBa();
        trang_thai_14 = CHAY_BINH_THUONG;
        dang_dung_vi_vat_can = false;
    }
    return false;
}

bool XuLyMatLine() {
    if (raw_sensor == 13) {
        if (!dang_mat_line) { dang_mat_line = true; thoi_gian_mat_line = millis(); }
        if (millis() - thoi_gian_mat_line >= thoi_gian_bo_qua_mat_line) {
            step_dc(false, false, HIGH, LOW, 0, 0);
            dang_dung_vi_mat_line = true;
            return true;
        }
        if (millis() - thoi_gian_mat_line >= time_bao_mat_line_lau) digitalWrite(pin_bao_mat_line, LOW);
    } else if (abs(raw_sensor) <= 2 || raw_sensor == 14 || raw_sensor == 15) {
        dang_mat_line = false;
        digitalWrite(pin_bao_mat_line, HIGH);
        if (dang_dung_vi_mat_line) { KhoiTaoThongSoDeBa(); dang_dung_vi_mat_line = false; }
    } else if (dang_dung_vi_mat_line) return true;
    return false;
}

void DocVaLocCamBien() {
    if ((raw_sensor == 14 || raw_sensor == 15) && trang_thai_14 == CHAY_BINH_THUONG) {
        if (abs(last_valid_sensor) >= 6 || abs(smoothedError) > 5.5) raw_sensor = (last_valid_sensor > 0) ? 9 : -9;
    }
    is_special_code = (raw_sensor == 12 || raw_sensor == 13 || raw_sensor == 14 || raw_sensor == 15);
    if (!is_special_code) {
        steering_error = last_valid_sensor = raw_sensor;
    } else {
        steering_error = (raw_sensor == 13) ? last_valid_sensor : 0;
    }
}

// --- HÀM QUẢN LÝ TỰ ĐỘNG TẮT CHÂN 11 VÀ 12 ---
void XuLyCacTinHieuPhu() {
    unsigned long now = millis();
    // Sau 1s (1000ms), tự động đưa chân 12 về lại HIGH
    if (dang_bat_pin_12 && (now - t_bat_dau_pin_12 >= 1000)) {
        digitalWrite(pin_tin_hieu_12, HIGH);
        dang_bat_pin_12 = false;
    }
    // Sau 1s (1000ms), tự động đưa chân 11 về lại HIGH
    if (dang_bat_pin_11 && (now - t_bat_dau_pin_11 >= 1000)) {
        digitalWrite(pin_tin_hieu_11, HIGH);
        dang_bat_pin_11 = false;
    }
}
// --------------------------------------------------------

void XuLyMayTrangThai() {
    unsigned long now = millis();
    
    // 1. CHỐN NHIỄU LÚC MỚI KHỞI ĐỘNG
    if (now - thoi_gian_cap_nguon < thoi_gian_tang_toc && (raw_sensor == 14 || raw_sensor == 15)) return;
    
    // 2. KỊCH BẢN QUAY ĐẦU (MÃ 12)
    if (raw_sensor == 12 && trang_thai_14 == CHAY_BINH_THUONG) {
        trang_thai_14 = QUAY_DAU; 
        thoi_gian_bat_dau_quay = now;
    }
    if (trang_thai_14 == QUAY_DAU) {
        if (now - thoi_gian_bat_dau_quay >= THOI_GIAN_QUAY_MU && raw_sensor == 0) {
            trang_thai_14 = CHAY_BINH_THUONG; 
            KhoiTaoThongSoDeBa();
        }
        return; 
    }

    // 3. KỊCH BẢN LÙI -> CHẠY MÙ (MÃ 15)
    if (trang_thai_14 == LUI_CASE_15) {
        if (now - t_bat_dau_lui_15 >= 10000) { // Lùi đủ 10s
            trang_thai_14 = TAM_DUNG_SAU_LUI;  
            t_bat_dau_dung_sau_lui = now;      
            
            digitalWrite(pin_tin_hieu_11, LOW); // Mở xi lanh
            dang_bat_pin_11 = true;
            t_bat_dau_pin_11 = now;
        }
        return; 
    }

    if (trang_thai_14 == TAM_DUNG_SAU_LUI) {
        if (now - t_bat_dau_dung_sau_lui >= 1000) { // Đã dừng đủ 1s
            trang_thai_14 = CHAY_BINH_THUONG;
            KhoiTaoThongSoDeBa(); 
            
            dang_chay_mu_sau_lui = true; // Kích hoạt chạy mù 23s
            t_ket_thuc_lui = now;
        }
        return; 
    }
    
    // Tắt khiên chạy mù nếu hết 23s
    if (dang_chay_mu_sau_lui && (now - t_ket_thuc_lui >= time_chay_mu_sau_lui)) {
        dang_chay_mu_sau_lui = false; 
    }

    // 4. KỊCH BẢN ĐỌC VẠCH 14 (Có bọc chống chạy mù)
    if (!dang_chay_mu_sau_lui) {
        // Vừa qua vạch 14, 15 thì phải cách 4s mới được đọc lại
        if (raw_sensor == 14 && (now - thoi_gian_roi_tram >= 4000)) {
            // Lần 1 đọc thấy vạch 14
            if (trang_thai_14 == CHAY_BINH_THUONG) {
                trang_thai_14 = BAT_DAU_GIAM_TOC; 
                t_chong_doi_tram = now;
                dang_tang_toc = false; 
                dang_giam_toc = true;
                thoi_gian_bat_dau_giam = now; 
                toc_do_khi_bat_dau_giam = smoothBaseSpeed;
            } 
            // Lần 2 đọc thấy vạch 14 (Dừng RF)
            else if (trang_thai_14 == CHO_DUNG && (now - t_chong_doi_tram > 500)) {
                trang_thai_14 = DANG_DUNG_LAY_HANG; 
                t_dung = now;
                digitalWrite(pin_tin_hieu_RF, LOW); // Phát sóng RF
            }
        }
        // Đọc thấy vạch 15 ngay sau vạch 14
        else if (raw_sensor == 15 && trang_thai_14 == CHO_DUNG && (now - t_chong_doi_tram > 500)) {
            trang_thai_14 = LUI_CASE_15;
            t_bat_dau_lui_15 = now;
            
            digitalWrite(pin_tin_hieu_12, LOW); // Đóng xi lanh trước khi lùi
            dang_bat_pin_12 = true;
            t_bat_dau_pin_12 = now;
        }
    }

    // Chuyển từ giảm tốc sang chờ dừng khi đã rời vạch 14 lần 1
    if (trang_thai_14 == BAT_DAU_GIAM_TOC && !is_special_code && (now - t_chong_doi_tram > 150)) {
        trang_thai_14 = CHO_DUNG;
    }
    
    // ==========================================================
    // 5. KỊCH BẢN XẢ HÀNG (Dừng 1s -> Kích chân 11 -> Đợi 1s -> Chạy đi -> 3s sau rút xi lanh)
    // ==========================================================
    
    // Khi hết 8 giây chờ tín hiệu RF, ép xe vào trạng thái dừng (khóa bánh)
    if ((trang_thai_14 == BAT_DAU_GIAM_TOC || trang_thai_14 == CHO_DUNG) && (now - thoi_gian_bat_dau_giam > time_tang_toc_case14_lan_1)) {
        trang_thai_14 = TAM_DUNG_DE_THA; 
        t_bat_dau_tam_dung = now;
        buoc_11_done = false; 
    }

    if (trang_thai_14 == TAM_DUNG_DE_THA) {
        unsigned long tg_da_dung = now - t_bat_dau_tam_dung;
        
        // BƯỚC 1: Dừng đủ 1s (1000ms) -> Kích mở xi lanh (chân 11)
        if (tg_da_dung >= 1000 && !buoc_11_done) {
            digitalWrite(pin_tin_hieu_11, LOW);
            dang_bat_pin_11 = true;
            t_bat_dau_pin_11 = now;
            
            // Bật bộ đếm ngầm 3 giây sau sẽ rút xi lanh
            dang_cho_dong_xi_lanh_khi_chay = true;
            t_bat_dau_cho_dong = now;
            
            buoc_11_done = true; 
        }

        // BƯỚC 2: 1s sau khi kích chân 11 (Tức là tổng tg_da_dung = 2000ms) -> Đề ba chạy đi
        if (tg_da_dung >= 3000) {
            
            // BẬT KHIÊN: "Bịt mắt" cảm biến an toàn trong 2 giây để xe băng qua cục hàng mượt mà
            dang_mien_nhiem_an_toan = true;
            t_bat_dau_mien_nhiem = now;

            // Ép xe ĐỀ BA CHẠY TIẾP
            trang_thai_14 = CHAY_BINH_THUONG;
            KhoiTaoThongSoDeBa(); 
            thoi_gian_roi_tram = now;
        }
    }
    
    // --- TIẾN TRÌNH CHẠY NGẦM: CHỜ 3S (từ lúc mở) ĐỂ ĐÓNG XI LANH ---
    if (dang_cho_dong_xi_lanh_khi_chay && (now - t_bat_dau_cho_dong >= time_dung_12)) { // time_dung_12 = 3000
        digitalWrite(pin_tin_hieu_12, LOW); // Đóng xi lanh
        dang_bat_pin_12 = true;
        t_bat_dau_pin_12 = now;
        
        dang_cho_dong_xi_lanh_khi_chay = false; // Đóng xong thì tắt cờ đếm ngầm
    }
    // ==========================================================
    
    // 6. KỊCH BẢN DỪNG RF TRẠM LẤY HÀNG (2 VẠCH 14)
    if (trang_thai_14 == DANG_DUNG_LAY_HANG) {
        // Tắt sóng RF sau 3s
        if (now - t_dung >= tg_gui_tinh_hieu_rf) {
            digitalWrite(pin_tin_hieu_RF, HIGH);
        }
        // Hết 20s dừng -> Chuẩn bị rời trạm
        if (now - t_dung >= time_dung_lay_hang) {
            trang_thai_14 = ROI_KHOI_TRAM; 
            KhoiTaoThongSoDeBa();
        }
    }
    
    // Rời khỏi vạch 14 cuối cùng sau khi lấy hàng xong
    if (trang_thai_14 == ROI_KHOI_TRAM && !is_special_code) {
        trang_thai_14 = CHAY_BINH_THUONG;
        thoi_gian_roi_tram = now;
    }
}
void QuyetDinhXuatXung() {
    // Đã thêm TAM_DUNG_SAU_LUI vào đây để lúc đợi 2s xe khoá bánh đứng cứng ngắc
    if (trang_thai_14 == DANG_DUNG_LAY_HANG || trang_thai_14 == TAM_DUNG_DE_THA || trang_thai_14 == TAM_DUNG_SAU_LUI) {
        step_dc(false, false, HIGH, LOW, 0, 0); last_ramp_time = millis(); return;
    }
    
    if (trang_thai_14 == LUI_CASE_15) {
        step_dc(true, true, LOW, HIGH, toc_do_lui_15, toc_do_lui_15); last_ramp_time = millis(); return;
    }
    
    if (trang_thai_14 == QUAY_DAU) {
        step_dc(true, true, LOW, LOW, speed_quay_dau, speed_quay_dau); last_ramp_time = millis(); return;
    }
    
    TinhToanBiendangTocDo();
    TinhToanVaXuatPID();
}

void TinhToanBiendangTocDo() {
    if (dang_tang_toc) {
        unsigned long t_chay = millis() - thoi_gian_bat_dau;
        if (t_chay < thoi_gian_tang_toc) {
            currentSpeedVal = toc_do_khi_bat_dau_tang - ((toc_do_khi_bat_dau_tang - speed) * ((float)t_chay / thoi_gian_tang_toc));
        } else {
            currentSpeedVal = (float)speed; dang_tang_toc = false;
        }
    } else if (dang_giam_toc) {
        unsigned long t_giam = millis() - thoi_gian_bat_dau_giam;
        if (t_giam < thoi_gian_giam_toc) {
            currentSpeedVal = toc_do_khi_bat_dau_giam + ((slowSpeed - toc_do_khi_bat_dau_giam) * ((float)t_giam / thoi_gian_giam_toc));
        } else {
            currentSpeedVal = (float)slowSpeed; dang_giam_toc = false;
        }
    }
    smoothBaseSpeed = (currentSpeedVal * 0.1) + (smoothBaseSpeed * 0.9);
}

void TinhToanVaXuatPID() {
    int baseSpeed = (int)smoothBaseSpeed;
    smoothedError = ((float)steering_error * 0.35) + (smoothedError * 0.65);
    float target_Kp, target_Kd;
    int target_offset_raw;
    int absError = abs(steering_error);
    
    if (absError == 0)      { target_Kp = Kp_123; target_Kd = Kd_123; target_offset_raw = 0; }
    else if (absError <= 3) { target_Kp = Kp_123; target_Kd = Kd_123; target_offset_raw = BU_ZONE_123; }
    else if (absError <= 6) { target_Kp = Kp_456; target_Kd = Kd_456; target_offset_raw = BU_ZONE_456; }
    else                    { target_Kp = Kp_789; target_Kd = Kd_789; target_offset_raw = BU_ZONE_789; }
    
    float speed_ratio = constrain((float)(startSpeed - baseSpeed) / (startSpeed - speed), 0.0, 1.0);
    int target_offset = (int)(target_offset_raw * speed_ratio);
    static float current_Kp = Kp_123, current_Kd = Kd_123, current_offset = 0.0;
    int UPDATE_RATE = 5;
    
    if (millis() - last_ramp_time >= UPDATE_RATE) {
        last_ramp_time = millis();
        float time_vao = max(TIME_VAO_CUA, 1.0f), time_ra = max(TIME_RA_CUA, 1.0f);
        float step_offset_vao = ((float)BU_ZONE_789 / time_vao) * UPDATE_RATE;
        float step_offset_ra  = ((float)BU_ZONE_789 / time_ra) * UPDATE_RATE;
        float step_Kp_vao = ((Kp_789 - Kp_123) / time_vao) * UPDATE_RATE, step_Kp_ra = ((Kp_789 - Kp_123) / time_ra) * UPDATE_RATE;
        float step_Kd_vao = ((Kd_789 - Kd_123) / time_vao) * UPDATE_RATE, step_Kd_ra = ((Kd_789 - Kd_123) / time_ra) * UPDATE_RATE;
        
        if (current_offset < target_offset) current_offset = min(current_offset + step_offset_vao, (float)target_offset);
        else if (current_offset > target_offset) current_offset = max(current_offset - step_offset_ra, (float)target_offset);
        if (current_Kp < target_Kp) current_Kp = min(current_Kp + step_Kp_vao, target_Kp);
        else if (current_Kp > target_Kp) current_Kp = max(current_Kp - step_Kp_ra, target_Kp);
        if (current_Kd < target_Kd) current_Kd = min(current_Kd + step_Kd_vao, target_Kd);
        else if (current_Kd > target_Kd) current_Kd = max(current_Kd - step_Kd_ra, target_Kd);
    }
    
    int finalBaseSpeed = constrain(baseSpeed + (int)current_offset, speed, startSpeed);
    integral = constrain(integral + smoothedError, -100, 100);
    float derivative = constrain(smoothedError - lastError, -2.0, 2.0);
    float rawPid = (current_Kp * smoothedError) + (Ki * integral) + (current_Kd * derivative);
    lastError = smoothedError;
    
    float filteredPid = (rawPid * 0.05) + (lastPidOutput * 0.95);
    if (filteredPid > lastPidOutput + 100) filteredPid = lastPidOutput + 100;
    else if (filteredPid < lastPidOutput - 100) filteredPid = lastPidOutput - 100;
    lastPidOutput = filteredPid;
    
    int PHUC_DEPTRAI = constrain(finalBaseSpeed - (int)filteredPid, speed, startSpeed);
    int PHUC_KDEP = constrain(finalBaseSpeed + (int)filteredPid, speed, startSpeed);
    step_dc(true, true, HIGH, LOW, PHUC_KDEP, PHUC_DEPTRAI + 20);
}


mới ở đây fix tiếp
// AGV SHIZAI - BAN HOAN CHINH KICH PIN 10, 11, 12 (UPDATE LOGIC LÙI)
#include "control_step.h"
#include "sensor.h"

// --- KHAI BÁO CÁC CHÂN TÍN HIỆU ---
const int pin_bao_mat_line = 45;
const int pin_dung = 46;
const int pin_tin_hieu_RF = 44;
const int pin_tin_hieu_10 = 10;
const int pin_tin_hieu_11 = 11;
const int pin_tin_hieu_12 = 12;

const unsigned long time_bao_mat_line_lau = 10000;
const unsigned long thoi_gian_bo_qua_mat_line = 500;
const unsigned long time_hu_line = 50;
const int time_tang_toc_case14_lan_1 = 8000;
const unsigned long time_dung_lay_hang = 20000;
const int speed = 1230;
const int slowSpeed = 12000;
const int startSpeed = 12000;

const int BU_ZONE_123 = 0;
const int BU_ZONE_456 = 400;
const int BU_ZONE_789 = 500;
const int tg_gui_tinh_hieu_rf = 3000;

const unsigned long thoi_gian_tang_toc = 9000;
const unsigned long thoi_gian_giam_toc = 6000;
const float Kp_123 = 50.0; const float Kd_123 = 500.0;
const float Kp_456 = 150.0; const float Kd_456 = 500.0;
const float Kp_789 = 200.0; const float Kd_789 = 500.0;
const float Ki = 0.01;
const float TIME_VAO_CUA = 200.0;
const float TIME_RA_CUA  = 300.0;

// --- CHẾ ĐỘ LÙI CỦA SHIZAI --- 
const int toc_do_lui_15 = 6000;
const unsigned long time_lui_ve = 10000; // Thời gian lùi 10s
const unsigned long time_chay_mu_sau_lui = 23000; // 23s bỏ qua trạm
unsigned long t_bat_dau_lui_15 = 0;
unsigned long t_bat_dau_dung_sau_lui = 0;
unsigned long t_ket_thuc_lui = 0;
bool dang_chay_mu_sau_lui = false;

// Biến quản lý tự tắt chân 11 và 12 sau 1s
unsigned long t_bat_dau_pin_11 = 0;
bool dang_bat_pin_11 = false;
unsigned long t_bat_dau_pin_12 = 0;
bool dang_bat_pin_12 = false;

enum TrangThaiTram { 
    CHAY_BINH_THUONG=0, BAT_DAU_GIAM_TOC=1, CHO_DUNG=2, DANG_DUNG_LAY_HANG=3, 
    ROI_KHOI_TRAM=4, QUAY_DAU=5, TAM_DUNG_DE_THA=6, 
    LUI_CASE_15=7, TAM_DUNG_SAU_LUI=8 
};
TrangThaiTram trang_thai_14 = CHAY_BINH_THUONG;

unsigned long thoi_gian_cap_nguon = 0, thoi_gian_bat_dau = 0, thoi_gian_bat_dau_giam = 0, t_dung = 0;
unsigned long t_chong_doi_tram = 0, last_ramp_time = 0;
unsigned long thoi_gian_mat_line = 0, t_bat_dau_tam_dung = 0, thoi_gian_roi_tram = 0;
bool dang_mat_line = false, dang_dung_vi_vat_can = false, dang_dung_vi_mat_line = false;
bool dang_tang_toc = true, dang_giam_toc = false, is_special_code = false;
int raw_sensor = 0, steering_error = 0, last_valid_sensor = 0;
float toc_do_khi_bat_dau_giam = 0, toc_do_khi_bat_dau_tang = 0, currentSpeedVal = startSpeed, smoothBaseSpeed = 0;
float smoothedError = 0, lastError = 0, integral = 0, lastPidOutput = 0;

void setup() {
    pinMode(pin_dung, OUTPUT); digitalWrite(pin_dung, HIGH);
    pinMode(pin_bao_mat_line, OUTPUT); digitalWrite(pin_bao_mat_line, HIGH);
    pinMode(pin_tin_hieu_RF, OUTPUT); digitalWrite(pin_tin_hieu_RF, HIGH);
    
    // Cài đặt các chân tín hiệu mới
    pinMode(pin_tin_hieu_10, OUTPUT); digitalWrite(pin_tin_hieu_10, HIGH);
    pinMode(pin_tin_hieu_11, OUTPUT); digitalWrite(pin_tin_hieu_11, HIGH);
    pinMode(pin_tin_hieu_12, OUTPUT); digitalWrite(pin_tin_hieu_12, HIGH);

    pinMode(PUL_TRAI, OUTPUT); pinMode(DIR_TRAI, OUTPUT); pinMode(ENA_TRAI, OUTPUT);
    pinMode(PUL_PHAI, OUTPUT); pinMode(DIR_PHAI, OUTPUT); pinMode(ENA_PHAI, OUTPUT);
    pinMode(T4, INPUT); pinMode(T3, INPUT); pinMode(T2, INPUT); pinMode(T1, INPUT);
    pinMode(TG, INPUT); pinMode(PG, INPUT);
    pinMode(P1, INPUT); pinMode(P2, INPUT); pinMode(P3, INPUT); pinMode(P4, INPUT);
    pinMode(cb1, INPUT_PULLUP);
    digitalWrite(ENA_TRAI, LOW); digitalWrite(ENA_PHAI, LOW);
    
    thoi_gian_cap_nguon = millis();
    KhoiTaoThongSoDeBa();
    if (In_SenSor() == 14 || In_SenSor() == 15) trang_thai_14 = ROI_KHOI_TRAM;
}

void loop() {
    raw_sensor = In_SenSor();
    unsigned long now = millis();

    // Tự động tắt chân 11 (trả về HIGH) sau 1s
    if (dang_bat_pin_11 && (now - t_bat_dau_pin_11 >= 1000)) {
        digitalWrite(pin_tin_hieu_11, HIGH);
        dang_bat_pin_11 = false;
    }

    // Tự động tắt chân 12 (trả về HIGH) sau 1s
    if (dang_bat_pin_12 && (now - t_bat_dau_pin_12 >= 1000)) {
        digitalWrite(pin_tin_hieu_12, HIGH);
        dang_bat_pin_12 = false;
    }

    // Tự động tắt khiên mù sau 23s
    if (dang_chay_mu_sau_lui && (now - t_ket_thuc_lui >= time_chay_mu_sau_lui)) {
        dang_chay_mu_sau_lui = false;
    }

    bool dang_lui = (trang_thai_14 == LUI_CASE_15 || trang_thai_14 == TAM_DUNG_SAU_LUI);
    bool an_han_tim_line = (dang_chay_mu_sau_lui && (now - t_ket_thuc_lui < 2000)); // Ân hạn 2s cho PID vặn vào line
    
    if (!dang_lui && !an_han_tim_line) {
        if (KiemTraAnToanGap() || XuLyMatLine()) return;
    }
    
    DocVaLocCamBien();
    XuLyMayTrangThai();
    QuyetDinhXuatXung();
}

void KhoiTaoThongSoDeBa() {
    currentSpeedVal = smoothBaseSpeed = toc_do_khi_bat_dau_tang = (float)startSpeed;
    thoi_gian_bat_dau = last_ramp_time = thoi_gian_roi_tram = millis();
    dang_tang_toc = true; dang_giam_toc = false;
    integral = lastError = smoothedError = lastPidOutput = 0;
}

bool KiemTraAnToanGap() {
    if (digitalRead(cb1) == LOW) {
        step_dc(false, false, HIGH, LOW, 0, 0);
        dang_dung_vi_vat_can = true;
        return true;
    } else if (dang_dung_vi_vat_can) {
        KhoiTaoThongSoDeBa();
        trang_thai_14 = CHAY_BINH_THUONG;
        dang_dung_vi_vat_can = false;
    }
    return false;
}

bool XuLyMatLine() {
    if (raw_sensor == 13) {
        if (!dang_mat_line) { dang_mat_line = true; thoi_gian_mat_line = millis(); }
        if (millis() - thoi_gian_mat_line >= thoi_gian_bo_qua_mat_line) {
            step_dc(false, false, HIGH, LOW, 0, 0);
            dang_dung_vi_mat_line = true;
            return true;
        }
        if (millis() - thoi_gian_mat_line >= time_bao_mat_line_lau) digitalWrite(pin_bao_mat_line, LOW);
    } else if (abs(raw_sensor) <= 2 || raw_sensor == 14 || raw_sensor == 15) {
        dang_mat_line = false;
        digitalWrite(pin_bao_mat_line, HIGH);
        if (dang_dung_vi_mat_line) { KhoiTaoThongSoDeBa(); dang_dung_vi_mat_line = false; }
    } else if (dang_dung_vi_mat_line) return true;
    return false;
}

void DocVaLocCamBien() {
    if ((raw_sensor == 14 || raw_sensor == 15) && trang_thai_14 == CHAY_BINH_THUONG) {
        if (abs(last_valid_sensor) >= 6 || abs(smoothedError) > 5.5) raw_sensor = (last_valid_sensor > 0) ? 9 : -9;
    }
    
    is_special_code = (raw_sensor == 13 || raw_sensor == 14 || raw_sensor == 15);
    if (!is_special_code) {
        steering_error = last_valid_sensor = raw_sensor;
    } else {
        steering_error = (raw_sensor == 13) ? last_valid_sensor : 0;
    }
}

void XuLyMayTrangThai() {
    unsigned long now = millis();
    
    // =======================================================
    // KỊCH BẢN 15: KÍCH CHÂN 11 VÀ LÙI 10s -> KÍCH CHÂN 12 VÀ DỪNG 1s -> ĐỀ BA 23s
    // =======================================================
    if (raw_sensor == 15 && !dang_chay_mu_sau_lui && (trang_thai_14 == BAT_DAU_GIAM_TOC || trang_thai_14 == CHO_DUNG)) {
        trang_thai_14 = LUI_CASE_15;
        t_bat_dau_lui_15 = now;
        
        // Kích chân 11 ngay lúc vừa chạm 15 và bắt đầu lùi
        digitalWrite(pin_tin_hieu_11, LOW); 
        dang_bat_pin_11 = true;
        t_bat_dau_pin_11 = now;
        
        return; // Thoát ra ngay để chạy lùi
    }

    if (trang_thai_14 == LUI_CASE_15) {
        if (now - t_bat_dau_lui_15 >= time_lui_ve) { // Hết thời gian lùi (10s)
            trang_thai_14 = TAM_DUNG_SAU_LUI;
            t_bat_dau_dung_sau_lui = now;
            
            // Lùi xong -> kích chân 12 và bắt đầu nghỉ 1s
            digitalWrite(pin_tin_hieu_12, LOW); 
            dang_bat_pin_12 = true;
            t_bat_dau_pin_12 = now;
        }
        return;
    }

    if (trang_thai_14 == TAM_DUNG_SAU_LUI) {
        if (now - t_bat_dau_dung_sau_lui >= 1000) { // Hết 1s dừng
            trang_thai_14 = CHAY_BINH_THUONG;
            KhoiTaoThongSoDeBa(); // Đề ba vọt đi
            dang_chay_mu_sau_lui = true; // Bật khiên mù 23s
            t_ket_thuc_lui = now;
        }
        return;
    }

    // =======================================================
    // LOGIC CHẠY BÌNH THƯỜNG VÀ XẢ HÀNG CASE 14 (KÍCH CHÂN 10)
    // =======================================================
    if (now - thoi_gian_cap_nguon < thoi_gian_tang_toc && (raw_sensor == 14 || raw_sensor == 15)) return;
    
    if (raw_sensor == 14 && (now - thoi_gian_roi_tram >= 4000) && !dang_chay_mu_sau_lui) {
        if (trang_thai_14 == CHAY_BINH_THUONG) {
            trang_thai_14 = BAT_DAU_GIAM_TOC; t_chong_doi_tram = now;
            dang_tang_toc = false; dang_giam_toc = true;
            thoi_gian_bat_dau_giam = now; toc_do_khi_bat_dau_giam = smoothBaseSpeed;
        } else if (trang_thai_14 == CHO_DUNG && (now - t_chong_doi_tram > 500)) {
            trang_thai_14 = DANG_DUNG_LAY_HANG; t_dung = now;
            digitalWrite(pin_tin_hieu_RF, LOW);
        }
    }
    
    if (trang_thai_14 == BAT_DAU_GIAM_TOC && !is_special_code && (now - t_chong_doi_tram > 150)) {
        trang_thai_14 = CHO_DUNG;
    }
    
    if ((trang_thai_14 == BAT_DAU_GIAM_TOC || trang_thai_14 == CHO_DUNG) && (now - thoi_gian_bat_dau_giam > time_tang_toc_case14_lan_1)) {
        trang_thai_14 = TAM_DUNG_DE_THA; 
        t_bat_dau_tam_dung = now;
        digitalWrite(pin_tin_hieu_10, LOW); // Kích chân 10 khi xe dừng
    }
    
    if (trang_thai_14 == TAM_DUNG_DE_THA) {
        if (now - t_bat_dau_tam_dung >= 1000) { 
            digitalWrite(pin_tin_hieu_10, HIGH); // Sau đúng 1s thì tắt chân 10
        }
        if (now - t_bat_dau_tam_dung >= 1200) {  // Chờ thêm 200ms để nhả xong tín hiệu rồi đề ba
            trang_thai_14 = CHAY_BINH_THUONG;
            dang_giam_toc = false; dang_tang_toc = true;
            toc_do_khi_bat_dau_tang = smoothBaseSpeed; 
            thoi_gian_bat_dau = now;
            thoi_gian_roi_tram = now;
        }
    }
    
    if (trang_thai_14 == DANG_DUNG_LAY_HANG) {
        if (now - t_dung >= tg_gui_tinh_hieu_rf) digitalWrite(pin_tin_hieu_RF, HIGH);
        if (now - t_dung >= time_dung_lay_hang) {
            trang_thai_14 = ROI_KHOI_TRAM; KhoiTaoThongSoDeBa();
        }
    }
    
    if (trang_thai_14 == ROI_KHOI_TRAM && !is_special_code) {
        trang_thai_14 = CHAY_BINH_THUONG;
        thoi_gian_roi_tram = now;
    }
}

void QuyetDinhXuatXung() {
    // NGẮT ĐỘNG CƠ CHO CÁC TRẠNG THÁI DỪNG
    if (trang_thai_14 == DANG_DUNG_LAY_HANG || trang_thai_14 == TAM_DUNG_DE_THA || trang_thai_14 == TAM_DUNG_SAU_LUI) {
        step_dc(false, false, HIGH, LOW, 0, 0); last_ramp_time = millis(); return;
    }
    
    // XUẤT XUNG LÙI CỦA CASE 15
    if (trang_thai_14 == LUI_CASE_15) {
        step_dc(true, true, LOW, HIGH, toc_do_lui_15, toc_do_lui_15); last_ramp_time = millis(); return;
    }
    
    TinhToanBiendangTocDo();
    TinhToanVaXuatPID();
}

void TinhToanBiendangTocDo() {
    if (dang_tang_toc) {
        unsigned long t_chay = millis() - thoi_gian_bat_dau;
        if (t_chay < thoi_gian_tang_toc) {
            currentSpeedVal = toc_do_khi_bat_dau_tang - ((toc_do_khi_bat_dau_tang - speed) * ((float)t_chay / thoi_gian_tang_toc));
        } else {
            currentSpeedVal = (float)speed; dang_tang_toc = false;
        }
    } else if (dang_giam_toc) {
        unsigned long t_giam = millis() - thoi_gian_bat_dau_giam;
        if (t_giam < thoi_gian_giam_toc) {
            currentSpeedVal = toc_do_khi_bat_dau_giam + ((slowSpeed - toc_do_khi_bat_dau_giam) * ((float)t_giam / thoi_gian_giam_toc));
        } else {
            currentSpeedVal = (float)slowSpeed; dang_giam_toc = false;
        }
    }
    smoothBaseSpeed = (currentSpeedVal * 0.1) + (smoothBaseSpeed * 0.9);
}

void TinhToanVaXuatPID() {
    int baseSpeed = (int)smoothBaseSpeed;
    smoothedError = ((float)steering_error * 0.35) + (smoothedError * 0.65);
    float target_Kp, target_Kd;
    int target_offset_raw;
    int absError = abs(steering_error);
    
    if (absError == 0)      { target_Kp = Kp_123; target_Kd = Kd_123; target_offset_raw = 0; }
    else if (absError <= 3) { target_Kp = Kp_123; target_Kd = Kd_123; target_offset_raw = BU_ZONE_123; }
    else if (absError <= 6) { target_Kp = Kp_456; target_Kd = Kd_456; target_offset_raw = BU_ZONE_456; }
    else                    { target_Kp = Kp_789; target_Kd = Kd_789; target_offset_raw = BU_ZONE_789; }
    
    float speed_ratio = constrain((float)(startSpeed - baseSpeed) / (startSpeed - speed), 0.0, 1.0);
    int target_offset = (int)(target_offset_raw * speed_ratio);
    static float current_Kp = Kp_123, current_Kd = Kd_123, current_offset = 0.0;
    int UPDATE_RATE = 5;
    
    if (millis() - last_ramp_time >= UPDATE_RATE) {
        last_ramp_time = millis();
        float time_vao = max(TIME_VAO_CUA, 1.0f), time_ra = max(TIME_RA_CUA, 1.0f);
        float step_offset_vao = ((float)BU_ZONE_789 / time_vao) * UPDATE_RATE;
        float step_offset_ra  = ((float)BU_ZONE_789 / time_ra) * UPDATE_RATE;
        float step_Kp_vao = ((Kp_789 - Kp_123) / time_vao) * UPDATE_RATE, step_Kp_ra = ((Kp_789 - Kp_123) / time_ra) * UPDATE_RATE;
        float step_Kd_vao = ((Kd_789 - Kd_123) / time_vao) * UPDATE_RATE, step_Kd_ra = ((Kd_789 - Kd_123) / time_ra) * UPDATE_RATE;
        
        if (current_offset < target_offset) current_offset = min(current_offset + step_offset_vao, (float)target_offset);
        else if (current_offset > target_offset) current_offset = max(current_offset - step_offset_ra, (float)target_offset);
        if (current_Kp < target_Kp) current_Kp = min(current_Kp + step_Kp_vao, target_Kp);
        else if (current_Kp > target_Kp) current_Kp = max(current_Kp - step_Kp_ra, target_Kp);
        if (current_Kd < target_Kd) current_Kd = min(current_Kd + step_Kd_vao, target_Kd);
        else if (current_Kd > target_Kd) current_Kd = max(current_Kd - step_Kd_ra, target_Kd);
    }
    
    int finalBaseSpeed = constrain(baseSpeed + (int)current_offset, speed, startSpeed);
    integral = constrain(integral + smoothedError, -100, 100);
    float derivative = constrain(smoothedError - lastError, -2.0, 2.0);
    float rawPid = (current_Kp * smoothedError) + (Ki * integral) + (current_Kd * derivative);
    lastError = smoothedError;
    
    float filteredPid = (rawPid * 0.05) + (lastPidOutput * 0.95);
    if (filteredPid > lastPidOutput + 100) filteredPid = lastPidOutput + 100;
    else if (filteredPid < lastPidOutput - 100) filteredPid = lastPidOutput - 100;
    lastPidOutput = filteredPid;
    
    int PHUC_DEPTRAI = constrain(finalBaseSpeed - (int)filteredPid, speed, startSpeed);
    int PHUC_KDEP = constrain(finalBaseSpeed + (int)filteredPid, speed, startSpeed);
    step_dc(true, true, HIGH, LOW, PHUC_KDEP, PHUC_DEPTRAI + 25);
}


karakuri o dây 
#include <AccelStepper.h>

#define PUL_PIN A0
#define DIR_PIN A1
#define ENA_PIN A2

#define CAM_BIEN_12 12
#define PIN_TIEN    24  // Nhận tín hiệu từ chân 11 AGV để vươn lên
#define PIN_LUI     22  // Nhận tín hiệu từ chân 12 AGV để thu về
#define PIN_TU_DONG 26

AccelStepper stepper(1, PUL_PIN, DIR_PIN);

const float STEPS_PER_REV = 1000.0;
float GOC_DO = 550.0; 

float TOC_DO_TIM_GOC = 500.0; 
float TOC_DO_LAM_VIEC = 1200.0;  
float GIA_TOC_LAM_VIEC = 1500.0; 
const unsigned long THOI_GIAN_CHONG_NHIEU = 10; 

bool daTimDuocGoc = false; 
unsigned long thoiGianBatDauLow = 0;
bool dangXacNhanGoc = false;

// Quản lý trạng thái Tự động (Chân 26)
enum TrangThaiTuDong { IDLE, DANG_TIEN, DANG_CHO_5S, DANG_LUI };
TrangThaiTuDong trangThaiTD = IDLE;
unsigned long thoiGianBatDauCho = 0;

void setup() {
  pinMode(CAM_BIEN_12, INPUT_PULLUP);
  pinMode(PIN_TIEN, INPUT_PULLUP);
  pinMode(PIN_LUI, INPUT_PULLUP);
  pinMode(PIN_TU_DONG, INPUT_PULLUP);

  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW); 

  stepper.setMaxSpeed(2000.0); 
}

void loop() {

  // --------------------------------------------------------
  // TRẠNG THÁI 1: MỞ NGUỒN -> DÒ TÌM ĐIỂM GỐC (CẢM BIẾN 12)
  // --------------------------------------------------------
  if (!daTimDuocGoc) {
    stepper.setSpeed(TOC_DO_TIM_GOC);
    stepper.runSpeed(); 

    if (digitalRead(CAM_BIEN_12) == LOW) {
      if (!dangXacNhanGoc) {
        dangXacNhanGoc = true;
        thoiGianBatDauLow = millis();
      } else if (millis() - thoiGianBatDauLow >= THOI_GIAN_CHONG_NHIEU) {
        stepper.stop(); 
        stepper.setCurrentPosition(0); // LƯU VỊ TRÍ NÀY LÀ MỐC 0
        
        stepper.setMaxSpeed(TOC_DO_LAM_VIEC); 
        stepper.setAcceleration(GIA_TOC_LAM_VIEC);
        
        daTimDuocGoc = true; 
        dangXacNhanGoc = false;
        trangThaiTD = IDLE;
      }
    } else {
      dangXacNhanGoc = false;
    }
  } 

  // --------------------------------------------------------
  // TRẠNG THÁI 2: ĐÃ CÓ GỐC -> LÀM VIỆC (CHÂN 22, 24, 26)
  // --------------------------------------------------------
  else {
    long xungLamViec = (GOC_DO * STEPS_PER_REV) / 360.0; 

    // NÚT 22: KÍCH CHẠY TỚI VỊ TRÍ ĐẶT
    if (digitalRead(PIN_TIEN) == LOW) {
      trangThaiTD = IDLE; // Hủy tự động nếu bấm tay
      stepper.moveTo(xungLamViec);
    }

    // NÚT 24: KÍCH CHẠY LÙI MƯỢT VỀ GỐC 0
    if (digitalRead(PIN_LUI) == LOW) {
      trangThaiTD = IDLE; // Hủy tự động nếu bấm tay
      stepper.moveTo(0);  // Quay về vị trí 0 bằng gia tốc làm việc
    }

    // NÚT 26: KÍCH CHẾ ĐỘ TỰ ĐỘNG (Tiến -> Đợi 5s -> Lùi)
    if (digitalRead(PIN_TU_DONG) == LOW && trangThaiTD == IDLE) {
      stepper.moveTo(xungLamViec);
      trangThaiTD = DANG_TIEN;
    }

    // XỬ LÝ CHU TRÌNH TỰ ĐỘNG (CHÂN 26)
    switch (trangThaiTD) {
      case DANG_TIEN:
        // Kiểm tra đã chạy tới góc đặt chưa
        if (stepper.distanceToGo() == 0) {
          thoiGianBatDauCho = millis();
          trangThaiTD = DANG_CHO_5S;
        }
        break;

      case DANG_CHO_5S:
        // Đếm đủ 5 giây
        if (millis() - thoiGianBatDauCho >= 5000) {
          stepper.moveTo(0); // Lệnh lùi về vị trí 0
          trangThaiTD = DANG_LUI;
        }
        break;

      case DANG_LUI:
        // Kiểm tra đã lùi về tới gốc 0 chưa
        if (stepper.distanceToGo() == 0) {
          trangThaiTD = IDLE; // Hoàn thành chu trình tự động
        }
        break;

      default:
        break;
    }

    // Lệnh bắt buộc để động cơ di chuyển theo gia tốc
    stepper.run(); 
  }
}
