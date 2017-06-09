#include <unit_test/unit_test.h>
#include <version/version.h>

class VersioningTest : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool _is_correct_version_tag(const char *version_tag, uint32_t result_goal);
	bool _is_correct_version_tag_vendor(const char *version_tag, uint32_t result_goal);

	bool _test_flight_version();
	bool _test_vendor_version();
};

bool VersioningTest::_is_correct_version_tag(const char *version_tag, uint32_t result_goal)
{
	if (version_tag_to_number(version_tag) == result_goal) {
		return true;

	} else {
		return false;
	}
}

bool VersioningTest::_is_correct_version_tag_vendor(const char *version_tag, uint32_t result_goal)
{
	if (version_tag_to_vendor_version_number(version_tag) == result_goal) {
		return true;

	} else {
		return false;
	}
}

bool VersioningTest::run_tests()
{
	ut_run_test(_test_flight_version);
	ut_run_test(_test_vendor_version);

	return (_tests_failed == 0);
}

bool VersioningTest::_test_flight_version()
{
	ut_assert_true(_is_correct_version_tag("v11.45.99-1.2.3", 0xB2D63ff));
	ut_assert_true(_is_correct_version_tag("v1.2.3", 0x010203ff));
	ut_assert_true(_is_correct_version_tag("v1.2.3-11", 0x01020300));
	ut_assert_true(_is_correct_version_tag("v1.2.3-11-abababab", 0x01020300));
	ut_assert_true(_is_correct_version_tag("11.45.99-1.2.3", 0x0B2D63ff));
	ut_assert_true(_is_correct_version_tag("v11.45.99-1.2.3rc3-7-g7e282f57", 0x0B2D6300));
	ut_assert_true(_is_correct_version_tag("v11.45.99-1.2.3rc4", 0x0B2D63C0));
	ut_assert_true(_is_correct_version_tag("v11.45.99-1.2.3alpha3-7-g7e282f57", 0x0B2D6300));
	ut_assert_true(_is_correct_version_tag("v11.45.99-1.2.3alpha4", 0x0B2D6340));
	ut_assert_true(_is_correct_version_tag("v11.45.99-1.2.3beta3-7-g7e282f57", 0x0B2D6300));
	ut_assert_true(_is_correct_version_tag("v11.45.99-1.2.3beta4", 0x0B2D6380));
	ut_assert_true(_is_correct_version_tag("v11.45.99-1.2.3dev4", 0x0B2D6300));
	ut_assert_true(_is_correct_version_tag("v11.45.99-1.2.3dev3-7-g7e282f57", 0xB2D6300));
	ut_assert_true(_is_correct_version_tag("0.45.99-1.2.3beta4", 0x002D6380));
	ut_assert_true(_is_correct_version_tag("0.0.0-1.2.3beta4", 0x00000080));
	ut_assert_true(_is_correct_version_tag("0.0.0-1.2.3dev4", 0x00000000));

	return true;
}

bool VersioningTest::_test_vendor_version()
{
	ut_assert_true(_is_correct_version_tag_vendor("alpha", 0x000000));
	ut_assert_true(_is_correct_version_tag_vendor("alpha23", 0x000000));
	ut_assert_true(_is_correct_version_tag_vendor("v11.45.99-34.56.88", 0x223858));
	ut_assert_true(_is_correct_version_tag_vendor("v11.45.99-1.2.3", 0x010203));
	ut_assert_true(_is_correct_version_tag_vendor("1.2.3-11.45.99", 0x0B2D63));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3", 0x000000));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-11", 0x000000));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-11.45", 0x000000));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-11-abababab", 0x000000));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-11.45.99rc3-7-g7e282f57", 0x0B2D63));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-11.45.99rc4", 0x0B2D63));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-11.45.99alpha3-7-g7e282f57", 0x0B2D63));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-11.45.99alpha4", 0x0B2D63));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-11.45.99beta3-7-g7e282f57", 0x0B2D63));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-11.45.99beta4", 0x0B2D63));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-11.45.99dev4", 0x0B2D63));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-11.45.99dev3-7-g7e282f57", 0x0B2D63));
	ut_assert_true(_is_correct_version_tag_vendor("1.2.3-0.45.99beta4", 0x002D63));
	ut_assert_true(_is_correct_version_tag_vendor("1.2.3-0.0.0beta4", 0x000000));
	ut_assert_true(_is_correct_version_tag_vendor("1.2.3-0.0.0dev4", 0x000000));

	return true;
}

ut_declare_test_c(test_versioning, VersioningTest);
