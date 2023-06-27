#include <stdio.h>

#define No_Calibration_Point 5

struct CalibrationData {
    double X[No_Calibration_Point];
    double y[No_Calibration_Point];
};

double linear_mapping(double x, struct CalibrationData cal_data)
{
    int i = 0;
    double y;

    // X범위 아래 값

    if (x <= cal_data.X[0]) {
        y = cal_data.y[i] + (cal_data.y[i + 1] - cal_data.y[i]) * (x - cal_data.X[i]) / (cal_data.X[i + 1] - cal_data.X[i]);
    }
    // 범위 이상 값
    else if (x >= cal_data.X[No_Calibration_Point - 1]) {
        i = No_Calibration_Point - 2;
        y = cal_data.y[i] + (cal_data.y[i + 1] - cal_data.y[i]) * (x - cal_data.X[i]) / (cal_data.X[i + 1] - cal_data.X[i]);
    }
    //정상 범위 값 
    else {
        y = cal_data.y[i] + (cal_data.y[i + 1] - cal_data.y[i]) * (x - cal_data.X[i]) / (cal_data.X[i + 1] - cal_data.X[i]);
    }
    

    return y;
}

int main()
{
    struct CalibrationData cal_data = {
        {0.0, 1.0, 2.0, 3.0, 4.0},  // X 범위
        {10.0, 20.0, 30.0, 40.0, 50.0}  // y 범위
    };

    double x =5;
    double mapped_value = linear_mapping(x, cal_data);
    printf("Mapped value: %.2f\n", mapped_value);

    return 0;
}

