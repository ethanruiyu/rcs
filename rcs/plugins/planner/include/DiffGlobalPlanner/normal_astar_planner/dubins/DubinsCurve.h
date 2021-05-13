//
// Created by yurui on 20-9-10.
//

#ifndef NORMAL_ASTAR_PLANNER_NODE_DUBINSCURVE_H
#define NORMAL_ASTAR_PLANNER_NODE_DUBINSCURVE_H


// Path types
#define LSL (0)
#define LSR (1)
#define RSL (2)
#define RSR (3)
#define RLR (4)
#define LRL (5)


// Error codes
#define EDUBOK        (0)   // No error
#define EDUBCOCONFIGS (1)   // Colocated configurations
#define EDUBPARAM     (2)   // Path parameterisitation error
#define EDUBBADRHO    (3)   // the rho value is invalid
#define EDUBNOPATH    (4)   // no connection between configurations with this word

typedef int (*DubinsWord)(double, double, double, double* );

// A complete list of the possible solvers that could give optimal paths
extern DubinsWord dubins_words[];

typedef struct
{
    double qi[3];       // the initial configuration
    double param[3];    // the lengths of the three segments
    double rho;         // model forward velocity / model angular velocity
    int type;           // path type. one of LSL, LSR, ...
} DubinsPath;

#define EPSILON (10e-10)


// The three segment types a path can be made up of
#define L_SEG (0)
#define S_SEG (1)
#define R_SEG (2)

#define UNPACK_INPUTS(alpha, beta)     \
    double sa = sin(alpha);            \
    double sb = sin(beta);             \
    double ca = cos(alpha);            \
    double cb = cos(beta);             \
    double c_ab = cos(alpha - beta);   \

#define PACK_OUTPUTS(outputs)       \
    outputs[0]  = t;                \
    outputs[1]  = p;                \
    outputs[2]  = q;


const int DIRDATA[][3] = {
        { L_SEG, S_SEG, L_SEG },
        { L_SEG, S_SEG, R_SEG },
        { R_SEG, S_SEG, L_SEG },
        { R_SEG, S_SEG, R_SEG },
        { R_SEG, L_SEG, R_SEG },
        { L_SEG, R_SEG, L_SEG }
};

double fmodr( double x, double y);
double mod2pi( double theta );
int dubins_init_normalised( double alpha, double beta, double d, DubinsPath* path);
int dubins_init( double q0[3], double q1[3], double rho, DubinsPath* path );
double dubins_path_length( DubinsPath* path );
void dubins_segment( double t, double qi[3], double qt[3], int type);
int dubins_path_sample( DubinsPath* path, double t, double q[3] );
int dubins_LSL( double alpha, double beta, double d, double* outputs );
int dubins_RSR( double alpha, double beta, double d, double* outputs );
int dubins_LSR( double alpha, double beta, double d, double* outputs );
int dubins_RSL( double alpha, double beta, double d, double* outputs );
int dubins_RLR( double alpha, double beta, double d, double* outputs );
int dubins_LRL( double alpha, double beta, double d, double* outputs );



#endif //NORMAL_ASTAR_PLANNER_NODE_DUBINSCURVE_H
