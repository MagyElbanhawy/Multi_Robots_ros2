from pathlib import Path

from emrmf_experiments.metrics import (
    ABLATION_MODES,
    alignment_rmse,
    build_configurations,
    compute_theta,
    pose_rmse,
    summarize_raw_rows,
    latex_table,
)
from emrmf_experiments.validation_simulator import generate_validation_artifacts


def test_theta_matches_requested_formula():
    theta = compute_theta(error_norm=0.25, tau_e=0.5, p=3.0, gamma=0.3, delta_t=2.0)
    expected = max(0.0, 1.0 - (0.25 / 0.5) ** 3.0) * 2.718281828459045 ** (-0.3 * 2.0)
    assert abs(theta - expected) < 1e-9
    assert compute_theta(error_norm=2.0, tau_e=0.5, p=3.0, gamma=0.3, delta_t=0.5) == 0.0


def test_rmse_helpers_for_pose_and_alignment():
    assert pose_rmse([(0, 0, 0), (1, 1, 0)], [(0, 0, 0), (2, 1, 0)]) > 0.0
    assert alignment_rmse([(0, 0, 0), (1, 0, 0)], [(0, 0, 0), (1, 0, 0)], nearest_neighbor=False) == 0.0


def test_configurations_include_required_sweeps_and_modes():
    configs = build_configurations(
        modes=ABLATION_MODES,
        p_values=[2, 3, 4],
        gamma_values=[0.1, 0.3, 0.5, 1.0],
        delay_values=[0.0, 0.5, 2.0],
        packet_loss_values=[0.0, 0.10, 0.30],
        sensor_noise_values=[0.0],
        tau_e=0.5,
    )
    assert {config.ablation_mode for config in configs} == set(ABLATION_MODES)
    assert {config.p for config in configs} == {2.0, 3.0, 4.0}
    assert {config.delay_s for config in configs} == {0.0, 0.5, 2.0}
    assert {config.packet_loss for config in configs} == {0.0, 0.10, 0.30}


def test_validation_generator_writes_raw_summary_markdown_and_latex(tmp_path: Path):
    result = generate_validation_artifacts(
        output_dir=tmp_path,
        modes=['baseline_graph_slam', 'full_emrmf'],
        p_values=[2, 3],
        gamma_values=[0.1],
        delay_values=[0.5, 2.0],
        packet_loss_values=[0.10, 0.30],
        sensor_noise_values=[0.10, 0.20, 0.30],
        repeated_runs=5,
        tau_e=0.5,
        seed=7,
    )
    assert Path(result['raw_csv']).exists()
    assert Path(result['summary_csv']).exists()
    assert Path(result['summary_markdown']).exists()
    assert Path(result['summary_latex']).exists()
    assert len(result['rows']) == 2 * 2 * 1 * 2 * 2 * 3 * 5
    assert all(row['runs'] == 5 for row in result['summary_rows'])
    assert '95% confidence interval' in Path(result['summary_markdown']).read_text(encoding='utf-8')
    assert '\\begin{table}' in Path(result['summary_latex']).read_text(encoding='utf-8')


def test_latex_table_escapes_modes_and_terminates_rows():
    rows = summarize_raw_rows([
        {
            'timestamp': '2026-06-03T00:00:00Z',
            'configuration_id': 'demo',
            'run_id': 1,
            'ablation_mode': 'baseline_graph_slam',
            'p': 3.0,
            'gamma': 0.3,
            'tau_e': 0.5,
            'delay_s': 0.5,
            'packet_loss': 0.1,
            'sensor_noise': 0.2,
            'pose_rmse': 0.1,
            'map_alignment_rmse': 0.2,
            'fusion_time_ms': 10.0,
            'theta_mean': 0.8,
            'theta_std': 0.05,
            'false_constraint_acceptance_rate': 0.01,
            'map_consistency_score': 0.9,
            'accepted_constraints': 42,
            'success': 1,
        }
    ])
    latex = latex_table(rows)
    assert r'baseline\_graph\_slam' in latex
    assert r'0.800 \\' in latex
