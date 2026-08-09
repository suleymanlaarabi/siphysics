
/* A friendly warning from bake.test
 * ----------------------------------------------------------------------------
 * This file is generated. To add/remove testcases modify the 'project.json' of
 * the test project. ANY CHANGE TO THIS FILE IS LOST AFTER (RE)BUILDING!
 * ----------------------------------------------------------------------------
 */

#include <test.h>

// Testsuite 'basic'
void basic_simple(void);

bake_test_case basic_testcases[] = {
    {
        "simple",
        basic_simple
    }
};


static bake_test_suite suites[] = {
    {
        "basic",
        NULL,
        NULL,
        1,
        basic_testcases
    }
};

int main(int argc, char *argv[]) {
    return bake_test_run("siphysics.test", argc, argv, suites, 1);
}
