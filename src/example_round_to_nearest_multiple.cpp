#include <bits/stdc++.h>

float roundMultiple(float number, float multiple)
{
    float result = round(number / multiple) * multiple;
    std::cout << "Number: " << number << ", Nearest: " << result << std::endl;
    return result;
}

int main(int argc, char *argv)
{
    roundMultiple(4.2, 0.25);
    roundMultiple(4.26, 0.25);
    roundMultiple(4.4, 0.25);
    roundMultiple(-4.2, 0.25);
    roundMultiple(-4.26, 0.25);
    roundMultiple(-4.4, 0.25);

    return 0;
}