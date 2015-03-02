
class TwoLink {
public:
    TwoLink();
    
    void dae(double* derivatives, double* path, double* states, 
            double* controls, double* parameters, double& time, 
            double* xad, int iphase);

};
