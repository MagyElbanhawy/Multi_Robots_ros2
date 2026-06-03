from emrmf_core.scalability import generate_scalability_summary, markdown_table


def test_scalability_summary_has_requested_robot_counts_and_columns():
    result = generate_scalability_summary(trials=3, samples_per_robot=40)
    rows = result['rows']
    assert [row['robot_count'] for row in rows] == [2, 3, 4, 5]
    for row in rows:
        assert row['mean_pose_rmse'] > 0.0
        assert row['std'] >= 0.0
        assert row['ci95'] >= 0.0
        assert row['alignment_rmse'] > 0.0
        assert row['fusion_time_ms'] > 0.0
        assert 0.0 <= row['mean_theta'] <= 1.0
        assert row['accepted_constraints'] > 0
        assert 0.0 <= row['success_rate'] <= 1.0


def test_scalability_markdown_mentions_success_rate():
    result = generate_scalability_summary(trials=3, samples_per_robot=40)
    report = markdown_table(result)
    assert 'Success rate' in report
    assert '| 5 |' in report
