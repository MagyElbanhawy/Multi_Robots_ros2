from emrmf_core.robustness import generate_robustness_comparison, markdown_table


def test_robustness_comparison_includes_all_stressors_and_exponents():
    result = generate_robustness_comparison(runs=3, samples_per_run=50)
    assert result['stressors'] == ['packet_loss', 'delay', 'noise']
    assert result['exponents'] == [2.0, 3.0, 4.0]
    assert len(result['rows']) == 27
    assert result['recommended_exponent'] == 3.0


def test_robustness_markdown_mentions_cubic_recommendation():
    result = generate_robustness_comparison(runs=3, samples_per_run=50)
    report = markdown_table(result)
    assert 'Recommended exponent: **p=3**' in report
    assert '| packet_loss |' in report
    assert '| delay |' in report
    assert '| noise |' in report
