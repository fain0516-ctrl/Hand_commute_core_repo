"""STS3215 를 공개 식별값(Rhoban BAM, feetech_sts3215_7_4V m1~m6)으로 바꿔 다시 계산.

출처: https://github.com/Rhoban/bam  (bam/feetech/actuator.py, bam/params/feetech_sts3215_7_4V/*.json)
BAM 의 STS3215 펌웨어 모델 (오실로스코프로 측정, 7.4 V):
  duty = clamp((q_target_smooth - q) * P * error_gain * error_gain_ratio, +-0.97),  P = P 계수 레지스터(기본 32), error_gain = 0.166 /rad
  V = vin * duty,  tau = kt * (V - kt * dq) / R,  J = armature,  + 점성 friction_viscous + 쿨롱 friction_base
  q_target_smooth 는 max_velocity(~5.2 rad/s) 로 속도 제한, 명령 지연 command_delay
선형화 (포화, 속도 제한, 쿨롱 마찰 제외):
  J s^2 q + B s q = K e^{-s tau} (q_cmd - q),  K = kt*vin*P*eg*ratio/R,  B = kt^2/R + viscous
"""
import glob, json, os
import numpy as np
import control as ct
import loop_model as lm

HERE = os.path.dirname(os.path.abspath(__file__))
VIN, EG, P_DEFAULT = 7.4, 0.166, 32

def load_models():
    out = []
    for f in sorted(glob.glob(os.path.join(HERE, "bam_params_sts3215_7_4V", "*.json"))):
        out.append(json.load(open(f)))
    return out

def plant(m, P=P_DEFAULT, j_load=0.0):
    K = m["kt"] * VIN * P * EG * m["error_gain_ratio"] / m["R"]
    B = m["kt"] ** 2 / m["R"] + m["friction_viscous"]
    J = m["armature"] + j_load
    return K, B, J

def inner(m, P=P_DEFAULT, j_load=0.0, pade=6):
    K, B, J = plant(m, P, j_load)
    tau = m["command_delay"]
    L0 = ct.tf([K], [J, B, 0])
    num, den = ct.pade(tau, pade)
    L = L0 * ct.tf(num, den)
    cl = ct.feedback(L, 1)
    p = ct.poles(cl)
    dom = p[np.argsort(-p.real)][:2]          # 허수축에 가장 가까운 2개 = 지배 극점
    gm, pm, _, wc = ct.margin(L)
    wn, zeta = np.sqrt(K / J), B / (2 * np.sqrt(K * J))
    return dict(K=K, B=B, J=J, tau=tau, wn=wn, zeta=zeta, dom=dom, maxre=p.real.max(), GM=gm, PM=pm,
                fc_hz=wc / 2 / np.pi, P_max=P * gm, err_sat_deg=np.degrees(0.97 / (P * EG * m["error_gain_ratio"])))

def p_max_for_delay(m, tau, j_load=0.0):
    mm = dict(m, command_delay=tau)
    return inner(mm, j_load=j_load)["P_max"]

if __name__ == "__main__":
    np.set_printoptions(precision=3, suppress=True)
    models = load_models()
    print("# 1) 내부 서보 루프 (P=32 기본값, 부하 관성 0)")
    for m in models:
        r = inner(m)
        print(f"{m['model']}: K={r['K']:.2f}Nm/rad B={r['B']:.3f} J={r['J']:.4f} tau={r['tau']*1e3:.1f}ms "
              f"wn={r['wn']/2/np.pi:.2f}Hz zeta={r['zeta']:.2f} dom={r['dom']} GM={20*np.log10(r['GM']):.1f}dB "
              f"PM={r['PM']:.1f}deg P_max={r['P_max']:.0f} sat_err={r['err_sat_deg']:.1f}deg")
    print("\n# 2) P 계수 레지스터 / 부하 관성 / 명령 지연 바꿔 보기 (m1, m6)")
    for m in (models[0], models[-1]):
        for P in (16, 32, 64, 128):
            r = inner(m, P)
            print(f"{m['model']} P={P:3d}: zeta={r['zeta']:.2f} wn={r['wn']/2/np.pi:.2f}Hz maxRe={r['maxre']:8.2f} PM={r['PM']:6.1f}")
        for jl in (0.0, 0.01, 0.05, 0.2):
            r = inner(m, j_load=jl)
            print(f"{m['model']} J_load={jl:.2f}: zeta={r['zeta']:.2f} wn={r['wn']/2/np.pi:.2f}Hz maxRe={r['maxre']:8.2f} PM={r['PM']:6.1f} P_max={r['P_max']:.0f}")
        for tau in (0.001, 0.005, 0.01, 0.02):
            print(f"{m['model']} tau={tau*1e3:.0f}ms: P_max={p_max_for_delay(m, tau):.0f}")
    print("\n# 3) 전체 체인 + 외부 루프 (loop_model.report 에 BAM wn, zeta, 명령 지연 반영)")
    base_cmd = lm.TD_CMD
    for m in models:
        r = inner(m)
        lm.TD_CMD = base_cmd + r["tau"]
        o = lm.report(r["wn"], r["zeta"])
        print(f"{m['model']}: servo poles={o['servo_poles_s']} |z|max={np.abs(o['chain_poles_z']).max():.3f} "
              f"BW={o['bw_chain_hz']:.2f}Hz  outer Ki=5: GM={o['outer_GM_dB']:.1f}dB PM={o['outer_PM_deg']:.1f}deg "
              f"|zcl|max={o['outer_cl_max_abs_z']:.3f} Ki_max={o['outer_Ki_max']:.1f}/s")
    lm.TD_CMD = base_cmd
