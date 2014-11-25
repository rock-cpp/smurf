#include <boost/test/unit_test.hpp>
#include <smurf/Dummy.hpp>

using namespace smurf;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    smurf::DummyClass dummy;
    dummy.welcome();
}
