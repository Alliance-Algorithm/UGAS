#include "Ceres.h"

// Define the residual class for curve fitting
struct CurveFittingCost {
    CurveFittingCost(double t, double y)
        : t_(t), y_(y) {}

    template<typename T>
    bool operator()(const T* const parameters, T* residual) const {
        T a = parameters[0];
        T b = parameters[1];
        T c = parameters[2];
        T d = parameters[3];

        // Calculate the residual: y - (a * sin(b * t + c) + d)
        residual[0] = y_ - (a * ceres::sin(b * t_ + c) + d);
        return true;
    }

    static ceres::CostFunction* Create(double t, double y) {
        return new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 4>(
            new CurveFittingCost(t, y)
            );
    }

    double t_;
    double y_;
};

bool Ceres::Fitting()
{
    float rotateSum = 0;
    for (int i = 0; i < 40; i++)
        rotateSum += _data.read(i).speed;
    //std::cout<<rotateSum<<std::endl;
    if (rotateSum > 0) _rotateSign = 1;
    else _rotateSign = -1;

    // Read 't' and 'y' data from file
    std::vector<double> t_values;
    std::vector<double> y_values;

    TimeStamp t,t0;
    float t1;
    float y;
    t0 = _data.read(0).timeStamp;
    y = _data.read(0).speed * _rotateSign;

    t_values.push_back(0.0);
    y_values.push_back(y);



    for (int i = 1; i < _total_data; i++) {
        t = _data.read(i).timeStamp;
        y = _data.read(i).speed * _rotateSign;
        t -= t0;
        t1 = 0.001 * t;
        t_values.push_back(t1);
        y_values.push_back(y);
    }

    double* parameters;
    // Set the initial values for the parameters
    if(!is_params_confirmed) {
        double parameterstemp[4] = { 1.0, 1.0, 1.0, 1.0 };
        parameters = parameterstemp;
    }
    else {
        double parameterstemp[4] = {_lastBuffFitData.A, _lastBuffFitData.B, _lastBuffFitData.C, _lastBuffFitData.D };
        parameters = parameterstemp;
    }

    // Configure the Ceres problem
    ceres::Problem problem;
    for (size_t i = 0; i < t_values.size(); ++i) {
        problem.AddResidualBlock(
            CurveFittingCost::Create(t_values[i], y_values[i]),
            nullptr,
            parameters
        );
    }

    // Configure the solver options
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_type = ceres::TRUST_REGION;

    //设置上下限
    problem.SetParameterLowerBound(parameters, 0, 0.5);
    problem.SetParameterUpperBound(parameters, 0, 2);
    problem.SetParameterLowerBound(parameters, 1, 1);
    problem.SetParameterUpperBound(parameters, 1, 3);
    problem.SetParameterLowerBound(parameters, 2, -MathConsts::Pi);
    problem.SetParameterUpperBound(parameters, 2, MathConsts::Pi);

    // Solve the problem
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Print the optimization results
    //std::cout << summary.BriefReport() << std::endl;
    //std::cout << "Estimated parameters:";
    //for (const auto& param : parameters) {
        //std::cout << " " << param;
    //}
    //std::cout << std::endl;
    if (summary.final_cost < max_cost && !is_params_confirmed)
    {
        _lastBuffFitData.A = parameters[0];
        _lastBuffFitData.B = parameters[1];
        _lastBuffFitData.C = parameters[2];
        _lastBuffFitData.D = parameters[3];
        _lastBuffFitData.FitTime = _data.last().timeStamp;
        is_params_confirmed = true;
        _last_cost = summary.final_cost;
        _t0=t0;
        //cout << "Confirmed!" << endl;
    }
    else if (is_params_confirmed && summary.final_cost < _last_cost)
    {
        _lastBuffFitData.A = parameters[0];
        _lastBuffFitData.B = parameters[1];
        _lastBuffFitData.C = parameters[2];
        _lastBuffFitData.D = parameters[3];
        _lastBuffFitData.FitTime = _data.last().timeStamp;
        _last_cost = summary.final_cost;
        _t0=t0;
    }
    else
    {
        return is_params_confirmed;
    }
    std::cout << _lastBuffFitData.FitTime << "\t" << _lastBuffFitData.A << "\t" << _lastBuffFitData.B
              << "\t" << _lastBuffFitData.C << "\t" << _lastBuffFitData.D << std::endl;
    //std::cout<<"cost: "<<_last_cost<<std::endl;
    return true;
}

double Ceres::PredictDeltaAngle(TimeStamp now, TimeStamp predict_timeStamp) const
{
    float t1 = 0.001 * (now - _t0);
    float t2 = t1 + 0.001 * predict_timeStamp;
    float c1 = cos(_lastBuffFitData.B * t1 + _lastBuffFitData.C);
    float c2 = cos(_lastBuffFitData.B * t2 + _lastBuffFitData.C);
    float delta_angle = _lastBuffFitData.A / _lastBuffFitData.B * (c1 - c2) + _lastBuffFitData.D * 0.001 * predict_timeStamp;

    return delta_angle * _rotateSign;
}

double Ceres::shiftWindowFilter(BuffAngularSpeed buffSpeed)
{
    if (_total_data == MaxData_) {
        _data.pop();
        _total_data--;
    }
    _data.push_back(buffSpeed);
    _total_data++;
    //计算最大迭代次数
    auto start_idx = _data.size() - window_size - 1;
    auto max_iter = int(_data.size() - start_idx) - window_size + 1;

    if (max_iter <= 0 || start_idx < 0)
        return _data.last().speed;

    double total_sum = 0;
    // cout<<max_iter<<endl;
    for (int i = 0; i < max_iter; i++)
    {
        double sum = 0;
        for (int j = 0; j < window_size; j++)
            sum += _data.read(start_idx + i + j).speed;
        total_sum += sum / window_size;
    }
    auto filterSpeed = total_sum / max_iter;
    _data.pop_back();
    _data.push_back(BuffAngularSpeed(buffSpeed.timeStamp, filterSpeed));
    return filterSpeed;
}
