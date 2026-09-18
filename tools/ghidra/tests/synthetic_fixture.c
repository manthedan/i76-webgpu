/* Self-authored, asset-free ELF fixture for the real Ghidra smoke runner. */
#include <stdio.h>

const char synthetic_message[] = "synthetic-fixture-message";
const char synthetic_dash_message[] = "-literal-dash-search";

__attribute__((noinline)) int synthetic_add(int left, int right) {
    return left + right + 7;
}

__attribute__((noinline)) int synthetic_twice(int value) {
    int first = synthetic_add(value, 3);
    return synthetic_add(first, value);
}

int main(int argc, char **argv) {
    int result = synthetic_twice(argc);
    puts(synthetic_message);
    puts(synthetic_dash_message);
    if (argv == NULL) {
        return 99;
    }
    return result == 0x7fffffff;
}
