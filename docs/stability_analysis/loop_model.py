"""Pi 5 comm_core -> TCP -> Pico 2 -> STS3215 경로의 선형 모델과 극점/여유 계산.

[코드에서 읽은 값]  config/comm_core.yaml, pico_fw/config/controller.yaml, actuators.c, session.c
  Ts_pi    = 1/loop_rate_hz = 10 ms   Pi 루프 (지령 ZOH)
  Ts_bus   = loop_period_us = 2 ms    Pico core1 서보 루프 (지령 반영까지 최대 1 주기)
  N_sts=6, reads_per_loop=2           -> 축별 위치 갱신 6 ms 마다 (피드백 나이 최대 6 ms)
  STATE 는 ACTUATOR_CMD 를 받을 때마다 1회 송신 (session.c:149) -> 100 Hz
  UART 1 Mbaud, SYNC_WRITE 6축 x 6바이트 ~ 49 바이트 -> 0.49 ms
  Pi/Pico 둘 다 피드백 제어기 없음 (지령 통과 + 클램프), 필터 없음 (PR #4 현재 기준)
[가정한 값]
  STS3215 내부 위치 루프: 폐루프 2차 wn^2/(s^2+2*zeta*wn*s+wn^2), wn=2*pi*5 rad/s, zeta=0.7 (내부 PID 비공개)
  TCP/LAN + Python 송신: 1 ms,  UDP(CH1/CH2) 왕복 각 1 ms
  goal_speed=0, goal_acc=0 (최대) 이므로 큰 스텝은 속도 포화 -> 아래는 소신호 선형 모델
"""
import numpy as np
import control as ct

TS_PI, TS_BUS = 0.010, 0.002
T_TCP, T_UART, T_UDP = 0.001, 0.00049, 0.001
WN, ZETA = 2 * np.pi * 5.0, 0.7
KI_NOM = 5.0   # 가정: 외부 적분 이득 [1/s] (교차 ~0.8 Hz)

def servo(wn=WN, zeta=ZETA):
    return ct.tf([wn**2], [1, 2 * zeta * wn, wn**2])

# 지령 경로 순수 지연 (ZOH 제외): TCP + Pico 루프 대기(최악 1주기) + UART
TD_CMD = T_TCP + TS_BUS + T_UART
# 외부 루프(3-B 가 CH1 보고 CH2 로 지령) 를 닫을 때의 귀환 경로 지연 (최악)
TD_FB = (TS_BUS * 3) + TS_PI + T_UDP + TS_PI + T_UDP   # 위치 읽기 나이 + STATE->Pi 틱 + CH1 + Pi 다음 틱 픽업 + CH2

def zoh_freq(w, T):
    x = w * T / 2
    return np.exp(-1j * x) * np.sinc(x / np.pi)

def path_frd(w, G, td):
    """지령->실제위치 주파수응답: ZOH(Ts_pi) * 지연 * 서보"""
    return ct.frd(zoh_freq(w, TS_PI) * np.exp(-1j * w * td) * G(1j * w).squeeze() if callable(G) else
                  zoh_freq(w, TS_PI) * np.exp(-1j * w * td) * ct.evalfr(G, 1j * w), w)

def freq_resp(G, w):
    return np.array([complex(ct.evalfr(G, 1j * wi)) for wi in w])

def bw_hz(mag, w):
    idx = np.where(mag < 10 ** (-3 / 20))[0]
    return w[idx[0]] / 2 / np.pi if len(idx) else np.nan

def margins(L, w):
    """L: 복소 주파수응답 배열. GM(배), PM(deg), 교차 주파수(Hz)"""
    ph = np.unwrap(np.angle(L))
    mag = np.abs(L)
    gm = pm = wc = wg = np.nan
    i = np.where(np.diff(np.sign(ph + np.pi)))[0]
    if len(i):
        wg = w[i[0]]; gm = 1 / mag[i[0]]
    j = np.where(np.diff(np.sign(mag - 1)))[0]
    if len(j):
        wc = w[j[0]]; pm = np.degrees(ph[j[0]] + np.pi)
    return gm, pm, wc / 2 / np.pi, wg / 2 / np.pi

def discrete_chain(G, extra_delay_s):
    """Ts_pi 기준 ZOH 이산화 + 정수 샘플 지연(올림) -> z 영역 전달함수"""
    Gd = ct.c2d(G, TS_PI, "zoh")
    d = int(np.ceil(extra_delay_s / TS_PI - 1e-9))
    return Gd * ct.tf([1], [1] + [0] * d, TS_PI), d

def report(wn=WN, zeta=ZETA, verbose=True):
    G = servo(wn, zeta)
    w = np.logspace(-1, 3.5, 20000)
    out = {}
    # 1) 시스템 자체 (개루프 지령 경로): 극점 = 서보 극점, 지연/ZOH 는 극점을 만들지 않음
    out["servo_poles_s"] = ct.poles(G)
    out["servo_zeros_s"] = ct.zeros(G)
    Gd, d = discrete_chain(G, TD_CMD)
    out["chain_poles_z"] = ct.poles(Gd)
    out["chain_zeros_z"] = ct.zeros(Gd)
    out["chain_delay_samples"] = d
    Hc = zoh_freq(w, TS_PI) * np.exp(-1j * w * TD_CMD) * freq_resp(G, w)
    out["bw_servo_hz"] = bw_hz(np.abs(freq_resp(G, w)), w)
    out["bw_chain_hz"] = bw_hz(np.abs(Hc), w)
    # 서보 내부 루프의 등가 개루프 (wn^2/(s(s+2 zeta wn))) 여유
    Ls = ct.tf([wn**2], [1, 2 * zeta * wn, 0])
    gm, pm, wg, wp = ct.margin(Ls)
    out["servo_inner_PM_deg"] = pm
    # 2) 외부 루프 (가정): 3-B 가 CH1 위치를 보고 CH2 위치 지령을 적분 보정 C = Ki/s, Ki=KI_NOM
    TD_LOOP = TD_CMD + TD_FB
    L = (KI_NOM / (1j * w)) * Hc * np.exp(-1j * w * TD_FB)
    gm, pm, fc, fg = margins(L, w)
    out.update(loop_delay_ms=TD_LOOP * 1e3, outer_Ki=KI_NOM, outer_GM=gm, outer_GM_dB=20 * np.log10(gm),
               outer_PM_deg=pm, outer_fc_hz=fc, outer_f180_hz=fg)
    Ld, dl = discrete_chain(G, TD_LOOP)
    C = ct.tf([KI_NOM * TS_PI], [1, -1], TS_PI)
    CL = ct.feedback(C * Ld, 1)
    out["outer_cl_poles_z"] = ct.poles(CL)
    out["outer_cl_max_abs_z"] = np.max(np.abs(ct.poles(CL)))
    out["outer_loop_delay_samples"] = dl
    # 외부 적분 보정 (Ki) 의 안정 한계: C = Ki*Ts/(z-1)
    def stable_ki(ki):
        C = ct.tf([ki * TS_PI], [1, -1], TS_PI)
        return np.max(np.abs(ct.poles(ct.feedback(C * Ld, 1)))) < 1
    lo, hi = 0.0, 500.0
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if stable_ki(mid) else (lo, mid)
    out["outer_Ki_max"] = lo
    return out

if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    r = report()
    print(f"TD_CMD={TD_CMD*1e3:.2f} ms, TD_FB={TD_FB*1e3:.2f} ms")
    for k, v in r.items():
        print(f"{k}: {v}")
    print("\n# 민감도 (wn Hz, zeta) -> 서보 극점 실수부, 외부 Ki=5/s GM/PM, Ki_max")
    for f in (2, 5, 10, 20):
        for z in (0.3, 0.5, 0.7, 1.0):
            r = report(2 * np.pi * f, z)
            p = r["servo_poles_s"]
            print(f"wn={f:>2}Hz zeta={z:.1f}  Re(p)={p.real.max():8.2f}  |z|max={np.abs(r['chain_poles_z']).max():.3f}  "
                  f"BW={r['bw_chain_hz']:.2f}Hz  outer GM={r['outer_GM_dB']:6.2f}dB PM={r['outer_PM_deg']:6.1f}deg "
                  f"|zcl|max={r['outer_cl_max_abs_z']:.3f}  Ki_max={r['outer_Ki_max']:.1f}/s")
