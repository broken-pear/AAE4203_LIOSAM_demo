"""验证 sanity_check 新增的两条检查（真值轨迹过短 / 长度比异常）。

造一份 json + csv，直接调 sanity_check.main()，看它是否：
  1. 真值只有 1.2 m 时判 FAIL 并说出"车基本没动"
  2. 长度比 3.0 时给 WARN
  3. 正常数据（35 m、比值 1.02）不报这两条
"""
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SANITY = os.path.join(ROOT, 'experiments', 'sanity_check.py')

FAILS = []


def check(name, cond, detail=''):
    print(f'  [{"OK " if cond else "FAIL"}] {name} {detail}')
    if not cond:
        FAILS.append(name)


CSV_HEAD = ('scene,algorithm,frames,duration_s,ate_rmse_m,ate_mean_m,ate_median_m,'
            'ate_max_m,ate_rot_rmse_deg,raw_rmse_m,rpe_trans_pct_m,rpe_rot_deg_m,'
            'final_pos_err_m,final_yaw_err_deg,frames_over_0_5m,diverged,'
            'output_rate_hz,csv\n')


def write_case(tmp, scene, gt_len, ratio, ate, algo_rows=('loam', 'liosam')):
    tables = os.path.join(tmp, 'tables')
    jsons = os.path.join(tmp, 'json')
    os.makedirs(tables, exist_ok=True)
    os.makedirs(jsons, exist_ok=True)
    csv_path = os.path.join(tables, 'all_runs.csv')
    lines = [CSV_HEAD]
    runs = []
    for i, algo in enumerate(algo_rows):
        est_csv = f'/x/results/raw/{scene}/{algo}.csv'
        # 两条算法的数字必须不同，否则会触发"指标完全相同"那条 FAIL
        # （那是另一个设计好的检查，这里不想误触发）
        a = ate if algo == 'loam' else round(ate * 0.4, 4)
        lines.append(
            f'{scene},{algo},900,96.0,{a},{a},{a},{a * 1.2:.4f},1.0,'
            f'{a * 1.3:.4f},2.0,0.5,{a},1.0,0,false,9.8,{est_csv}\n')
        runs.append({
            'algo': algo,
            'csv': f'/x/results/raw/{scene}/{algo}.csv',
            'counts': {'est': 900, 'gt': 900},
            'association': {'mode': 'floor', 'match_fraction': 1.0},
            'ate': {'rmse': ate, 'mean': ate, 'median': ate, 'max': ate * 1.2},
            'path': {'gt_length_m': gt_len, 'est_length_m': gt_len * ratio,
                     'length_ratio': ratio, 'duration_s': 96.0},
            'warnings': [],
        })
    io.open(csv_path, 'w', encoding='utf-8').write(''.join(lines))
    io.open(os.path.join(jsons, f'{scene}.json'), 'w', encoding='utf-8').write(
        json.dumps(runs, indent=1))
    return csv_path, jsons


def run_sanity(csv_path, jsons):
    r = subprocess.run([sys.executable, SANITY, csv_path, '--json-dir', jsons],
                       capture_output=True, text=True, encoding='utf-8')
    return r.stdout + r.stderr, r.returncode


def main():
    tmp = tempfile.mkdtemp(prefix='sanity-')
    try:
        print('=== 1. 真值只有 1.2 m（车没动）===')
        csv_path, jsons = write_case(tmp, 's0_nominal', 1.2, 1.0, 0.01)
        out, rc = run_sanity(csv_path, jsons)
        check('判 FAIL', rc != 0, f'rc={rc}')
        check('说清是"车基本没动"', '车基本没动' in out)
        check('给出里程数', '1.20 m' in out, [l for l in out.splitlines()
                                            if '车基本没动' in l][:1])

        print('\n=== 2. 长度比 3.0（尺度不对）===')
        shutil.rmtree(os.path.join(tmp, 'json'))
        csv_path, jsons = write_case(tmp, 's0_nominal', 35.0, 3.0, 0.30)
        out, rc = run_sanity(csv_path, jsons)
        check('给 WARN 而不是 FAIL', '轨迹长度比' in out and rc == 0, f'rc={rc}')

        print('\n=== 3. 正常数据（35 m、比值 1.02）===')
        shutil.rmtree(os.path.join(tmp, 'json'))
        csv_path, jsons = write_case(tmp, 's0_nominal', 35.0, 1.02, 0.30)
        out, rc = run_sanity(csv_path, jsons)
        check('这两条都不报', '车基本没动' not in out and '轨迹长度比' not in out)
        check('整体通过', rc == 0, f'rc={rc}')

        print('\n=== 结论 ===')
        if FAILS:
            print(f'{len(FAILS)} FAILED: {FAILS}')
            return 1
        print('ALL PASS —— 真值过短会被判 FAIL，尺度异常会给 WARN，正常数据不误报')
        return 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
