from emrmf_core.ablation import generate_ablation_summary, markdown_table


def test_ablation_summary_contains_requested_methods_and_statistics():
    result = generate_ablation_summary(runs=3, samples_per_run=50)
    rows = {row['method']: row for row in result['rows']}
    assert list(rows) == ['baseline_graph_slam', 'decentralized_only', 'trust_only', 'full_emrmf']
    for row in rows.values():
        assert row['mean_rmse_m'] > 0.0
        assert row['std_rmse_m'] >= 0.0
        assert row['ci95_rmse_m'] >= 0.0
        assert row['mean_fusion_time_ms'] > 0.0
    assert result['best_method'] == 'full_emrmf'
    assert rows['full_emrmf']['mean_rmse_m'] < rows['baseline_graph_slam']['mean_rmse_m']


def test_ablation_markdown_has_final_summary_table_columns():
    result = generate_ablation_summary(runs=3, samples_per_run=50)
    report = markdown_table(result)
    assert '| Method | Mean RMSE (m) | Std RMSE (m) | 95% CI (m) | Fusion time (ms) |' in report
    assert '| full_emrmf |' in report
