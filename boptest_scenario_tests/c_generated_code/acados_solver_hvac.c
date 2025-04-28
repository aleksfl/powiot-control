/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

// standard
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
// acados
// #include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// example specific
#include "hvac_model/hvac_model.h"


#include "hvac_cost/hvac_cost.h"



#include "acados_solver_hvac.h"

#define NX     HVAC_NX
#define NZ     HVAC_NZ
#define NU     HVAC_NU
#define NP     HVAC_NP
#define NP_GLOBAL     HVAC_NP_GLOBAL
#define NY0    HVAC_NY0
#define NY     HVAC_NY
#define NYN    HVAC_NYN

#define NBX    HVAC_NBX
#define NBX0   HVAC_NBX0
#define NBU    HVAC_NBU
#define NG     HVAC_NG
#define NBXN   HVAC_NBXN
#define NGN    HVAC_NGN

#define NH     HVAC_NH
#define NHN    HVAC_NHN
#define NH0    HVAC_NH0
#define NPHI   HVAC_NPHI
#define NPHIN  HVAC_NPHIN
#define NPHI0  HVAC_NPHI0
#define NR     HVAC_NR

#define NS     HVAC_NS
#define NS0    HVAC_NS0
#define NSN    HVAC_NSN

#define NSBX   HVAC_NSBX
#define NSBU   HVAC_NSBU
#define NSH0   HVAC_NSH0
#define NSH    HVAC_NSH
#define NSHN   HVAC_NSHN
#define NSG    HVAC_NSG
#define NSPHI0 HVAC_NSPHI0
#define NSPHI  HVAC_NSPHI
#define NSPHIN HVAC_NSPHIN
#define NSGN   HVAC_NSGN
#define NSBXN  HVAC_NSBXN



// ** solver data **

hvac_solver_capsule * hvac_acados_create_capsule(void)
{
    void* capsule_mem = malloc(sizeof(hvac_solver_capsule));
    hvac_solver_capsule *capsule = (hvac_solver_capsule *) capsule_mem;

    return capsule;
}


int hvac_acados_free_capsule(hvac_solver_capsule *capsule)
{
    free(capsule);
    return 0;
}


int hvac_acados_create(hvac_solver_capsule* capsule)
{
    int N_shooting_intervals = HVAC_N;
    double* new_time_steps = NULL; // NULL -> don't alter the code generated time-steps
    return hvac_acados_create_with_discretization(capsule, N_shooting_intervals, new_time_steps);
}


int hvac_acados_update_time_steps(hvac_solver_capsule* capsule, int N, double* new_time_steps)
{
    if (N != capsule->nlp_solver_plan->N) {
        fprintf(stderr, "hvac_acados_update_time_steps: given number of time steps (= %d) " \
            "differs from the currently allocated number of " \
            "time steps (= %d)!\n" \
            "Please recreate with new discretization and provide a new vector of time_stamps!\n",
            N, capsule->nlp_solver_plan->N);
        return 1;
    }

    ocp_nlp_config * nlp_config = capsule->nlp_config;
    ocp_nlp_dims * nlp_dims = capsule->nlp_dims;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &new_time_steps[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &new_time_steps[i]);
    }
    return 0;
}

/**
 * Internal function for hvac_acados_create: step 1
 */
void hvac_acados_create_set_plan(ocp_nlp_plan_t* nlp_solver_plan, const int N)
{
    assert(N == nlp_solver_plan->N);

    /************************************************
    *  plan
    ************************************************/

    nlp_solver_plan->nlp_solver = SQP;

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
    nlp_solver_plan->relaxed_ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;

    nlp_solver_plan->nlp_cost[0] = EXTERNAL;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = EXTERNAL;

    nlp_solver_plan->nlp_cost[N] = LINEAR_LS;

    for (int i = 0; i < N; i++)
    {
        nlp_solver_plan->nlp_dynamics[i] = DISCRETE_MODEL;
        // discrete dynamics does not need sim solver option, this field is ignored
        nlp_solver_plan->sim_solver_plan[i].sim_solver = INVALID_SIM_SOLVER;
    }

    nlp_solver_plan->nlp_constraints[0] = BGH;

    for (int i = 1; i < N; i++)
    {
        nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;

    nlp_solver_plan->regularization = NO_REGULARIZE;

    nlp_solver_plan->globalization = FIXED_STEP;
}


static ocp_nlp_dims* hvac_acados_create_setup_dimensions(hvac_solver_capsule* capsule)
{
    ocp_nlp_plan_t* nlp_solver_plan = capsule->nlp_solver_plan;
    const int N = nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;

    /************************************************
    *  dimensions
    ************************************************/
    #define NINTNP1MEMS 18
    int* intNp1mem = (int*)malloc( (N+1)*sizeof(int)*NINTNP1MEMS );

    int* nx    = intNp1mem + (N+1)*0;
    int* nu    = intNp1mem + (N+1)*1;
    int* nbx   = intNp1mem + (N+1)*2;
    int* nbu   = intNp1mem + (N+1)*3;
    int* nsbx  = intNp1mem + (N+1)*4;
    int* nsbu  = intNp1mem + (N+1)*5;
    int* nsg   = intNp1mem + (N+1)*6;
    int* nsh   = intNp1mem + (N+1)*7;
    int* nsphi = intNp1mem + (N+1)*8;
    int* ns    = intNp1mem + (N+1)*9;
    int* ng    = intNp1mem + (N+1)*10;
    int* nh    = intNp1mem + (N+1)*11;
    int* nphi  = intNp1mem + (N+1)*12;
    int* nz    = intNp1mem + (N+1)*13;
    int* ny    = intNp1mem + (N+1)*14;
    int* nr    = intNp1mem + (N+1)*15;
    int* nbxe  = intNp1mem + (N+1)*16;
    int* np  = intNp1mem + (N+1)*17;

    for (int i = 0; i < N+1; i++)
    {
        // common
        nx[i]     = NX;
        nu[i]     = NU;
        nz[i]     = NZ;
        ns[i]     = NS;
        // cost
        ny[i]     = NY;
        // constraints
        nbx[i]    = NBX;
        nbu[i]    = NBU;
        nsbx[i]   = NSBX;
        nsbu[i]   = NSBU;
        nsg[i]    = NSG;
        nsh[i]    = NSH;
        nsphi[i]  = NSPHI;
        ng[i]     = NG;
        nh[i]     = NH;
        nphi[i]   = NPHI;
        nr[i]     = NR;
        nbxe[i]   = 0;
        np[i]     = NP;
    }

    // for initial state
    nbx[0] = NBX0;
    nsbx[0] = 0;
    ns[0] = NS0;
    nbxe[0] = 3;
    ny[0] = NY0;
    nh[0] = NH0;
    nsh[0] = NSH0;
    nsphi[0] = NSPHI0;
    nphi[0] = NPHI0;


    // terminal - common
    nu[N]   = 0;
    nz[N]   = 0;
    ns[N]   = NSN;
    // cost
    ny[N]   = NYN;
    // constraint
    nbx[N]   = NBXN;
    nbu[N]   = 0;
    ng[N]    = NGN;
    nh[N]    = NHN;
    nphi[N]  = NPHIN;
    nr[N]    = 0;

    nsbx[N]  = NSBXN;
    nsbu[N]  = 0;
    nsg[N]   = NSGN;
    nsh[N]   = NSHN;
    nsphi[N] = NSPHIN;

    /* create and set ocp_nlp_dims */
    ocp_nlp_dims * nlp_dims = ocp_nlp_dims_create(nlp_config);

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "np", np);

    ocp_nlp_dims_set_global(nlp_config, nlp_dims, "np_global", 0);
    ocp_nlp_dims_set_global(nlp_config, nlp_dims, "n_global_data", 0);

    for (int i = 0; i <= N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsg", &nsg[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbxe", &nbxe[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nh", &nh[0]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nsh", &nsh[0]);

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nh", &nh[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsh", &nsh[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, N, "ny", &ny[N]);

    free(intNp1mem);

    return nlp_dims;
}


/**
 * Internal function for hvac_acados_create: step 3
 */
void hvac_acados_create_setup_functions(hvac_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;

    /************************************************
    *  external functions
    ************************************************/

#define MAP_CASADI_FNC(__CAPSULE_FNC__, __MODEL_BASE_FNC__) do{ \
        capsule->__CAPSULE_FNC__.casadi_fun = & __MODEL_BASE_FNC__ ;\
        capsule->__CAPSULE_FNC__.casadi_n_in = & __MODEL_BASE_FNC__ ## _n_in; \
        capsule->__CAPSULE_FNC__.casadi_n_out = & __MODEL_BASE_FNC__ ## _n_out; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_in = & __MODEL_BASE_FNC__ ## _sparsity_in; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_out = & __MODEL_BASE_FNC__ ## _sparsity_out; \
        capsule->__CAPSULE_FNC__.casadi_work = & __MODEL_BASE_FNC__ ## _work; \
        external_function_external_param_casadi_create(&capsule->__CAPSULE_FNC__, &ext_fun_opts); \
    } while(false)

    external_function_opts ext_fun_opts;
    external_function_opts_set_to_default(&ext_fun_opts);


    ext_fun_opts.external_workspace = true;
    // external cost
    MAP_CASADI_FNC(ext_cost_0_fun, hvac_cost_ext_cost_0_fun);
    MAP_CASADI_FNC(ext_cost_0_fun_jac, hvac_cost_ext_cost_0_fun_jac);
    MAP_CASADI_FNC(ext_cost_0_fun_jac_hess, hvac_cost_ext_cost_0_fun_jac_hess);




    // discrete dynamics
    capsule->discr_dyn_phi_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
    for (int i = 0; i < N; i++)
    {
        MAP_CASADI_FNC(discr_dyn_phi_fun[i], hvac_dyn_disc_phi_fun);
    }

    capsule->discr_dyn_phi_fun_jac_ut_xt = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
    for (int i = 0; i < N; i++)
    {
        MAP_CASADI_FNC(discr_dyn_phi_fun_jac_ut_xt[i], hvac_dyn_disc_phi_fun_jac);
    }

  

  
    capsule->discr_dyn_phi_fun_jac_ut_xt_hess = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
    for (int i = 0; i < N; i++)
    {
        MAP_CASADI_FNC(discr_dyn_phi_fun_jac_ut_xt_hess[i], hvac_dyn_disc_phi_fun_jac_hess);
    }
    // external cost
    capsule->ext_cost_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
    for (int i = 0; i < N-1; i++)
    {
        MAP_CASADI_FNC(ext_cost_fun[i], hvac_cost_ext_cost_fun);
    }

    capsule->ext_cost_fun_jac = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
    for (int i = 0; i < N-1; i++)
    {
        MAP_CASADI_FNC(ext_cost_fun_jac[i], hvac_cost_ext_cost_fun_jac);
    }

    capsule->ext_cost_fun_jac_hess = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
    for (int i = 0; i < N-1; i++)
    {
        MAP_CASADI_FNC(ext_cost_fun_jac_hess[i], hvac_cost_ext_cost_fun_jac_hess);
    }

    

    

#undef MAP_CASADI_FNC
}


/**
 * Internal function for hvac_acados_create: step 4
 */
void hvac_acados_create_set_default_parameters(hvac_solver_capsule* capsule)
{

    const int N = capsule->nlp_solver_plan->N;
    // initialize parameters to nominal value
    double* p = calloc(NP, sizeof(double));
    p[0] = 10;
    p[2] = 0.01;

    for (int i = 0; i <= N; i++) {
        hvac_acados_update_params(capsule, i, p, NP);
    }
    free(p);


    // no global parameters defined
}


/**
 * Internal function for hvac_acados_create: step 5
 */
void hvac_acados_setup_nlp_in(hvac_solver_capsule* capsule, const int N, double* new_time_steps)
{
    assert(N == capsule->nlp_solver_plan->N);
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;

    int tmp_int = 0;

    /************************************************
    *  nlp_in
    ************************************************/
//    ocp_nlp_in * nlp_in = ocp_nlp_in_create(nlp_config, nlp_dims);
//    capsule->nlp_in = nlp_in;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    // set up time_steps and cost_scaling

    if (new_time_steps)
    {
        // NOTE: this sets scaling and time_steps
        hvac_acados_update_time_steps(capsule, N, new_time_steps);
    }
    else
    {
        // set time_steps
    double time_step = 300;
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_step);
        }
        // set cost scaling
        double* cost_scaling = malloc((N+1)*sizeof(double));
        cost_scaling[0] = 300;
        cost_scaling[1] = 300;
        cost_scaling[2] = 300;
        cost_scaling[3] = 300;
        cost_scaling[4] = 300;
        cost_scaling[5] = 300;
        cost_scaling[6] = 300;
        cost_scaling[7] = 300;
        cost_scaling[8] = 300;
        cost_scaling[9] = 300;
        cost_scaling[10] = 300;
        cost_scaling[11] = 300;
        cost_scaling[12] = 300;
        cost_scaling[13] = 300;
        cost_scaling[14] = 300;
        cost_scaling[15] = 300;
        cost_scaling[16] = 300;
        cost_scaling[17] = 300;
        cost_scaling[18] = 300;
        cost_scaling[19] = 300;
        cost_scaling[20] = 300;
        cost_scaling[21] = 300;
        cost_scaling[22] = 300;
        cost_scaling[23] = 300;
        cost_scaling[24] = 300;
        cost_scaling[25] = 300;
        cost_scaling[26] = 300;
        cost_scaling[27] = 300;
        cost_scaling[28] = 300;
        cost_scaling[29] = 300;
        cost_scaling[30] = 300;
        cost_scaling[31] = 300;
        cost_scaling[32] = 300;
        cost_scaling[33] = 300;
        cost_scaling[34] = 300;
        cost_scaling[35] = 300;
        cost_scaling[36] = 300;
        cost_scaling[37] = 300;
        cost_scaling[38] = 300;
        cost_scaling[39] = 300;
        cost_scaling[40] = 300;
        cost_scaling[41] = 300;
        cost_scaling[42] = 300;
        cost_scaling[43] = 300;
        cost_scaling[44] = 300;
        cost_scaling[45] = 300;
        cost_scaling[46] = 300;
        cost_scaling[47] = 300;
        cost_scaling[48] = 300;
        cost_scaling[49] = 300;
        cost_scaling[50] = 300;
        cost_scaling[51] = 300;
        cost_scaling[52] = 300;
        cost_scaling[53] = 300;
        cost_scaling[54] = 300;
        cost_scaling[55] = 300;
        cost_scaling[56] = 300;
        cost_scaling[57] = 300;
        cost_scaling[58] = 300;
        cost_scaling[59] = 300;
        cost_scaling[60] = 300;
        cost_scaling[61] = 300;
        cost_scaling[62] = 300;
        cost_scaling[63] = 300;
        cost_scaling[64] = 300;
        cost_scaling[65] = 300;
        cost_scaling[66] = 300;
        cost_scaling[67] = 300;
        cost_scaling[68] = 300;
        cost_scaling[69] = 300;
        cost_scaling[70] = 300;
        cost_scaling[71] = 300;
        cost_scaling[72] = 300;
        cost_scaling[73] = 300;
        cost_scaling[74] = 300;
        cost_scaling[75] = 300;
        cost_scaling[76] = 300;
        cost_scaling[77] = 300;
        cost_scaling[78] = 300;
        cost_scaling[79] = 300;
        cost_scaling[80] = 300;
        cost_scaling[81] = 300;
        cost_scaling[82] = 300;
        cost_scaling[83] = 300;
        cost_scaling[84] = 300;
        cost_scaling[85] = 300;
        cost_scaling[86] = 300;
        cost_scaling[87] = 300;
        cost_scaling[88] = 300;
        cost_scaling[89] = 300;
        cost_scaling[90] = 300;
        cost_scaling[91] = 300;
        cost_scaling[92] = 300;
        cost_scaling[93] = 300;
        cost_scaling[94] = 300;
        cost_scaling[95] = 300;
        cost_scaling[96] = 300;
        cost_scaling[97] = 300;
        cost_scaling[98] = 300;
        cost_scaling[99] = 300;
        cost_scaling[100] = 300;
        cost_scaling[101] = 300;
        cost_scaling[102] = 300;
        cost_scaling[103] = 300;
        cost_scaling[104] = 300;
        cost_scaling[105] = 300;
        cost_scaling[106] = 300;
        cost_scaling[107] = 300;
        cost_scaling[108] = 300;
        cost_scaling[109] = 300;
        cost_scaling[110] = 300;
        cost_scaling[111] = 300;
        cost_scaling[112] = 300;
        cost_scaling[113] = 300;
        cost_scaling[114] = 300;
        cost_scaling[115] = 300;
        cost_scaling[116] = 300;
        cost_scaling[117] = 300;
        cost_scaling[118] = 300;
        cost_scaling[119] = 300;
        cost_scaling[120] = 300;
        cost_scaling[121] = 300;
        cost_scaling[122] = 300;
        cost_scaling[123] = 300;
        cost_scaling[124] = 300;
        cost_scaling[125] = 300;
        cost_scaling[126] = 300;
        cost_scaling[127] = 300;
        cost_scaling[128] = 300;
        cost_scaling[129] = 300;
        cost_scaling[130] = 300;
        cost_scaling[131] = 300;
        cost_scaling[132] = 300;
        cost_scaling[133] = 300;
        cost_scaling[134] = 300;
        cost_scaling[135] = 300;
        cost_scaling[136] = 300;
        cost_scaling[137] = 300;
        cost_scaling[138] = 300;
        cost_scaling[139] = 300;
        cost_scaling[140] = 300;
        cost_scaling[141] = 300;
        cost_scaling[142] = 300;
        cost_scaling[143] = 300;
        cost_scaling[144] = 300;
        cost_scaling[145] = 300;
        cost_scaling[146] = 300;
        cost_scaling[147] = 300;
        cost_scaling[148] = 300;
        cost_scaling[149] = 300;
        cost_scaling[150] = 300;
        cost_scaling[151] = 300;
        cost_scaling[152] = 300;
        cost_scaling[153] = 300;
        cost_scaling[154] = 300;
        cost_scaling[155] = 300;
        cost_scaling[156] = 300;
        cost_scaling[157] = 300;
        cost_scaling[158] = 300;
        cost_scaling[159] = 300;
        cost_scaling[160] = 300;
        cost_scaling[161] = 300;
        cost_scaling[162] = 300;
        cost_scaling[163] = 300;
        cost_scaling[164] = 300;
        cost_scaling[165] = 300;
        cost_scaling[166] = 300;
        cost_scaling[167] = 300;
        cost_scaling[168] = 300;
        cost_scaling[169] = 300;
        cost_scaling[170] = 300;
        cost_scaling[171] = 300;
        cost_scaling[172] = 300;
        cost_scaling[173] = 300;
        cost_scaling[174] = 300;
        cost_scaling[175] = 300;
        cost_scaling[176] = 300;
        cost_scaling[177] = 300;
        cost_scaling[178] = 300;
        cost_scaling[179] = 300;
        cost_scaling[180] = 300;
        cost_scaling[181] = 300;
        cost_scaling[182] = 300;
        cost_scaling[183] = 300;
        cost_scaling[184] = 300;
        cost_scaling[185] = 300;
        cost_scaling[186] = 300;
        cost_scaling[187] = 300;
        cost_scaling[188] = 300;
        cost_scaling[189] = 300;
        cost_scaling[190] = 300;
        cost_scaling[191] = 300;
        cost_scaling[192] = 300;
        cost_scaling[193] = 300;
        cost_scaling[194] = 300;
        cost_scaling[195] = 300;
        cost_scaling[196] = 300;
        cost_scaling[197] = 300;
        cost_scaling[198] = 300;
        cost_scaling[199] = 300;
        cost_scaling[200] = 300;
        cost_scaling[201] = 300;
        cost_scaling[202] = 300;
        cost_scaling[203] = 300;
        cost_scaling[204] = 300;
        cost_scaling[205] = 300;
        cost_scaling[206] = 300;
        cost_scaling[207] = 300;
        cost_scaling[208] = 300;
        cost_scaling[209] = 300;
        cost_scaling[210] = 300;
        cost_scaling[211] = 300;
        cost_scaling[212] = 300;
        cost_scaling[213] = 300;
        cost_scaling[214] = 300;
        cost_scaling[215] = 300;
        cost_scaling[216] = 300;
        cost_scaling[217] = 300;
        cost_scaling[218] = 300;
        cost_scaling[219] = 300;
        cost_scaling[220] = 300;
        cost_scaling[221] = 300;
        cost_scaling[222] = 300;
        cost_scaling[223] = 300;
        cost_scaling[224] = 300;
        cost_scaling[225] = 300;
        cost_scaling[226] = 300;
        cost_scaling[227] = 300;
        cost_scaling[228] = 300;
        cost_scaling[229] = 300;
        cost_scaling[230] = 300;
        cost_scaling[231] = 300;
        cost_scaling[232] = 300;
        cost_scaling[233] = 300;
        cost_scaling[234] = 300;
        cost_scaling[235] = 300;
        cost_scaling[236] = 300;
        cost_scaling[237] = 300;
        cost_scaling[238] = 300;
        cost_scaling[239] = 300;
        cost_scaling[240] = 300;
        cost_scaling[241] = 300;
        cost_scaling[242] = 300;
        cost_scaling[243] = 300;
        cost_scaling[244] = 300;
        cost_scaling[245] = 300;
        cost_scaling[246] = 300;
        cost_scaling[247] = 300;
        cost_scaling[248] = 300;
        cost_scaling[249] = 300;
        cost_scaling[250] = 300;
        cost_scaling[251] = 300;
        cost_scaling[252] = 300;
        cost_scaling[253] = 300;
        cost_scaling[254] = 300;
        cost_scaling[255] = 300;
        cost_scaling[256] = 300;
        cost_scaling[257] = 300;
        cost_scaling[258] = 300;
        cost_scaling[259] = 300;
        cost_scaling[260] = 300;
        cost_scaling[261] = 300;
        cost_scaling[262] = 300;
        cost_scaling[263] = 300;
        cost_scaling[264] = 300;
        cost_scaling[265] = 300;
        cost_scaling[266] = 300;
        cost_scaling[267] = 300;
        cost_scaling[268] = 300;
        cost_scaling[269] = 300;
        cost_scaling[270] = 300;
        cost_scaling[271] = 300;
        cost_scaling[272] = 300;
        cost_scaling[273] = 300;
        cost_scaling[274] = 300;
        cost_scaling[275] = 300;
        cost_scaling[276] = 300;
        cost_scaling[277] = 300;
        cost_scaling[278] = 300;
        cost_scaling[279] = 300;
        cost_scaling[280] = 300;
        cost_scaling[281] = 300;
        cost_scaling[282] = 300;
        cost_scaling[283] = 300;
        cost_scaling[284] = 300;
        cost_scaling[285] = 300;
        cost_scaling[286] = 300;
        cost_scaling[287] = 300;
        cost_scaling[288] = 300;
        cost_scaling[289] = 300;
        cost_scaling[290] = 300;
        cost_scaling[291] = 300;
        cost_scaling[292] = 300;
        cost_scaling[293] = 300;
        cost_scaling[294] = 300;
        cost_scaling[295] = 300;
        cost_scaling[296] = 300;
        cost_scaling[297] = 300;
        cost_scaling[298] = 300;
        cost_scaling[299] = 300;
        cost_scaling[300] = 300;
        cost_scaling[301] = 300;
        cost_scaling[302] = 300;
        cost_scaling[303] = 300;
        cost_scaling[304] = 300;
        cost_scaling[305] = 300;
        cost_scaling[306] = 300;
        cost_scaling[307] = 300;
        cost_scaling[308] = 300;
        cost_scaling[309] = 300;
        cost_scaling[310] = 300;
        cost_scaling[311] = 300;
        cost_scaling[312] = 300;
        cost_scaling[313] = 300;
        cost_scaling[314] = 300;
        cost_scaling[315] = 300;
        cost_scaling[316] = 300;
        cost_scaling[317] = 300;
        cost_scaling[318] = 300;
        cost_scaling[319] = 300;
        cost_scaling[320] = 300;
        cost_scaling[321] = 300;
        cost_scaling[322] = 300;
        cost_scaling[323] = 300;
        cost_scaling[324] = 300;
        cost_scaling[325] = 300;
        cost_scaling[326] = 300;
        cost_scaling[327] = 300;
        cost_scaling[328] = 300;
        cost_scaling[329] = 300;
        cost_scaling[330] = 300;
        cost_scaling[331] = 300;
        cost_scaling[332] = 300;
        cost_scaling[333] = 300;
        cost_scaling[334] = 300;
        cost_scaling[335] = 300;
        cost_scaling[336] = 300;
        cost_scaling[337] = 300;
        cost_scaling[338] = 300;
        cost_scaling[339] = 300;
        cost_scaling[340] = 300;
        cost_scaling[341] = 300;
        cost_scaling[342] = 300;
        cost_scaling[343] = 300;
        cost_scaling[344] = 300;
        cost_scaling[345] = 300;
        cost_scaling[346] = 300;
        cost_scaling[347] = 300;
        cost_scaling[348] = 300;
        cost_scaling[349] = 300;
        cost_scaling[350] = 300;
        cost_scaling[351] = 300;
        cost_scaling[352] = 300;
        cost_scaling[353] = 300;
        cost_scaling[354] = 300;
        cost_scaling[355] = 300;
        cost_scaling[356] = 300;
        cost_scaling[357] = 300;
        cost_scaling[358] = 300;
        cost_scaling[359] = 300;
        cost_scaling[360] = 300;
        cost_scaling[361] = 300;
        cost_scaling[362] = 300;
        cost_scaling[363] = 300;
        cost_scaling[364] = 300;
        cost_scaling[365] = 300;
        cost_scaling[366] = 300;
        cost_scaling[367] = 300;
        cost_scaling[368] = 300;
        cost_scaling[369] = 300;
        cost_scaling[370] = 300;
        cost_scaling[371] = 300;
        cost_scaling[372] = 300;
        cost_scaling[373] = 300;
        cost_scaling[374] = 300;
        cost_scaling[375] = 300;
        cost_scaling[376] = 300;
        cost_scaling[377] = 300;
        cost_scaling[378] = 300;
        cost_scaling[379] = 300;
        cost_scaling[380] = 300;
        cost_scaling[381] = 300;
        cost_scaling[382] = 300;
        cost_scaling[383] = 300;
        cost_scaling[384] = 300;
        cost_scaling[385] = 300;
        cost_scaling[386] = 300;
        cost_scaling[387] = 300;
        cost_scaling[388] = 300;
        cost_scaling[389] = 300;
        cost_scaling[390] = 300;
        cost_scaling[391] = 300;
        cost_scaling[392] = 300;
        cost_scaling[393] = 300;
        cost_scaling[394] = 300;
        cost_scaling[395] = 300;
        cost_scaling[396] = 300;
        cost_scaling[397] = 300;
        cost_scaling[398] = 300;
        cost_scaling[399] = 300;
        cost_scaling[400] = 300;
        cost_scaling[401] = 300;
        cost_scaling[402] = 300;
        cost_scaling[403] = 300;
        cost_scaling[404] = 300;
        cost_scaling[405] = 300;
        cost_scaling[406] = 300;
        cost_scaling[407] = 300;
        cost_scaling[408] = 300;
        cost_scaling[409] = 300;
        cost_scaling[410] = 300;
        cost_scaling[411] = 300;
        cost_scaling[412] = 300;
        cost_scaling[413] = 300;
        cost_scaling[414] = 300;
        cost_scaling[415] = 300;
        cost_scaling[416] = 300;
        cost_scaling[417] = 300;
        cost_scaling[418] = 300;
        cost_scaling[419] = 300;
        cost_scaling[420] = 300;
        cost_scaling[421] = 300;
        cost_scaling[422] = 300;
        cost_scaling[423] = 300;
        cost_scaling[424] = 300;
        cost_scaling[425] = 300;
        cost_scaling[426] = 300;
        cost_scaling[427] = 300;
        cost_scaling[428] = 300;
        cost_scaling[429] = 300;
        cost_scaling[430] = 300;
        cost_scaling[431] = 300;
        cost_scaling[432] = 300;
        cost_scaling[433] = 300;
        cost_scaling[434] = 300;
        cost_scaling[435] = 300;
        cost_scaling[436] = 300;
        cost_scaling[437] = 300;
        cost_scaling[438] = 300;
        cost_scaling[439] = 300;
        cost_scaling[440] = 300;
        cost_scaling[441] = 300;
        cost_scaling[442] = 300;
        cost_scaling[443] = 300;
        cost_scaling[444] = 300;
        cost_scaling[445] = 300;
        cost_scaling[446] = 300;
        cost_scaling[447] = 300;
        cost_scaling[448] = 300;
        cost_scaling[449] = 300;
        cost_scaling[450] = 300;
        cost_scaling[451] = 300;
        cost_scaling[452] = 300;
        cost_scaling[453] = 300;
        cost_scaling[454] = 300;
        cost_scaling[455] = 300;
        cost_scaling[456] = 300;
        cost_scaling[457] = 300;
        cost_scaling[458] = 300;
        cost_scaling[459] = 300;
        cost_scaling[460] = 300;
        cost_scaling[461] = 300;
        cost_scaling[462] = 300;
        cost_scaling[463] = 300;
        cost_scaling[464] = 300;
        cost_scaling[465] = 300;
        cost_scaling[466] = 300;
        cost_scaling[467] = 300;
        cost_scaling[468] = 300;
        cost_scaling[469] = 300;
        cost_scaling[470] = 300;
        cost_scaling[471] = 300;
        cost_scaling[472] = 300;
        cost_scaling[473] = 300;
        cost_scaling[474] = 300;
        cost_scaling[475] = 300;
        cost_scaling[476] = 300;
        cost_scaling[477] = 300;
        cost_scaling[478] = 300;
        cost_scaling[479] = 300;
        cost_scaling[480] = 300;
        cost_scaling[481] = 300;
        cost_scaling[482] = 300;
        cost_scaling[483] = 300;
        cost_scaling[484] = 300;
        cost_scaling[485] = 300;
        cost_scaling[486] = 300;
        cost_scaling[487] = 300;
        cost_scaling[488] = 300;
        cost_scaling[489] = 300;
        cost_scaling[490] = 300;
        cost_scaling[491] = 300;
        cost_scaling[492] = 300;
        cost_scaling[493] = 300;
        cost_scaling[494] = 300;
        cost_scaling[495] = 300;
        cost_scaling[496] = 300;
        cost_scaling[497] = 300;
        cost_scaling[498] = 300;
        cost_scaling[499] = 300;
        cost_scaling[500] = 300;
        cost_scaling[501] = 300;
        cost_scaling[502] = 300;
        cost_scaling[503] = 300;
        cost_scaling[504] = 300;
        cost_scaling[505] = 300;
        cost_scaling[506] = 300;
        cost_scaling[507] = 300;
        cost_scaling[508] = 300;
        cost_scaling[509] = 300;
        cost_scaling[510] = 300;
        cost_scaling[511] = 300;
        cost_scaling[512] = 300;
        cost_scaling[513] = 300;
        cost_scaling[514] = 300;
        cost_scaling[515] = 300;
        cost_scaling[516] = 300;
        cost_scaling[517] = 300;
        cost_scaling[518] = 300;
        cost_scaling[519] = 300;
        cost_scaling[520] = 300;
        cost_scaling[521] = 300;
        cost_scaling[522] = 300;
        cost_scaling[523] = 300;
        cost_scaling[524] = 300;
        cost_scaling[525] = 300;
        cost_scaling[526] = 300;
        cost_scaling[527] = 300;
        cost_scaling[528] = 300;
        cost_scaling[529] = 300;
        cost_scaling[530] = 300;
        cost_scaling[531] = 300;
        cost_scaling[532] = 300;
        cost_scaling[533] = 300;
        cost_scaling[534] = 300;
        cost_scaling[535] = 300;
        cost_scaling[536] = 300;
        cost_scaling[537] = 300;
        cost_scaling[538] = 300;
        cost_scaling[539] = 300;
        cost_scaling[540] = 300;
        cost_scaling[541] = 300;
        cost_scaling[542] = 300;
        cost_scaling[543] = 300;
        cost_scaling[544] = 300;
        cost_scaling[545] = 300;
        cost_scaling[546] = 300;
        cost_scaling[547] = 300;
        cost_scaling[548] = 300;
        cost_scaling[549] = 300;
        cost_scaling[550] = 300;
        cost_scaling[551] = 300;
        cost_scaling[552] = 300;
        cost_scaling[553] = 300;
        cost_scaling[554] = 300;
        cost_scaling[555] = 300;
        cost_scaling[556] = 300;
        cost_scaling[557] = 300;
        cost_scaling[558] = 300;
        cost_scaling[559] = 300;
        cost_scaling[560] = 300;
        cost_scaling[561] = 300;
        cost_scaling[562] = 300;
        cost_scaling[563] = 300;
        cost_scaling[564] = 300;
        cost_scaling[565] = 300;
        cost_scaling[566] = 300;
        cost_scaling[567] = 300;
        cost_scaling[568] = 300;
        cost_scaling[569] = 300;
        cost_scaling[570] = 300;
        cost_scaling[571] = 300;
        cost_scaling[572] = 300;
        cost_scaling[573] = 300;
        cost_scaling[574] = 300;
        cost_scaling[575] = 300;
        cost_scaling[576] = 1;
        for (int i = 0; i <= N; i++)
        {
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &cost_scaling[i]);
        }
        free(cost_scaling);
    }


    /**** Dynamics ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "disc_dyn_fun", &capsule->discr_dyn_phi_fun[i]);
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "disc_dyn_fun_jac",
                                   &capsule->discr_dyn_phi_fun_jac_ut_xt[i]);
        
        
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "disc_dyn_fun_jac_hess",
                                   &capsule->discr_dyn_phi_fun_jac_ut_xt_hess[i]);
    }

    /**** Cost ****/
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun", &capsule->ext_cost_0_fun);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun_jac", &capsule->ext_cost_0_fun_jac);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun_jac_hess", &capsule->ext_cost_0_fun_jac_hess);
    
    
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun", &capsule->ext_cost_fun[i-1]);
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun_jac", &capsule->ext_cost_fun_jac[i-1]);
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun_jac_hess", &capsule->ext_cost_fun_jac_hess[i-1]);
        
        
    }



    // slacks
    double* zlumem = calloc(4*NS, sizeof(double));
    double* Zl = zlumem+NS*0;
    double* Zu = zlumem+NS*1;
    double* zl = zlumem+NS*2;
    double* zu = zlumem+NS*3;
    // change only the non-zero elements:
    zl[0] = 100;
    zu[0] = 100;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zl", Zl);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zu", Zu);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zl", zl);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zu", zu);
    }
    free(zlumem);



    /**** Constraints ****/

    // bounds for initial stage
    // x0
    int* idxbx0 = malloc(NBX0 * sizeof(int));
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;

    double* lubx0 = calloc(2*NBX0, sizeof(double));
    double* lbx0 = lubx0;
    double* ubx0 = lubx0 + NBX0;
    // change only the non-zero elements:
    lbx0[0] = 293.15;
    ubx0[0] = 293.15;
    lbx0[1] = 293.15;
    ubx0[1] = 293.15;
    lbx0[2] = 293.15;
    ubx0[2] = 293.15;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);
    free(idxbx0);
    free(lubx0);
    // idxbxe_0
    int* idxbxe_0 = malloc(3 * sizeof(int));
    idxbxe_0[0] = 0;
    idxbxe_0[1] = 1;
    idxbxe_0[2] = 2;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbxe", idxbxe_0);
    free(idxbxe_0);








    /* constraints that are the same for initial and intermediate */

    // ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxsbx", idxsbx);
    // ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lsbx", lsbx);
    // ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "usbx", usbx);

    // soft bounds on x
    int* idxsbx = malloc(NSBX * sizeof(int));
    idxsbx[0] = 0;

    double* lusbx = calloc(2*NSBX, sizeof(double));
    double* lsbx = lusbx;
    double* usbx = lusbx + NSBX;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxsbx", idxsbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lsbx", lsbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "usbx", usbx);
    }
    free(idxsbx);
    free(lusbx);
    // u
    int* idxbu = malloc(NBU * sizeof(int));
    idxbu[0] = 0;
    double* lubu = calloc(2*NBU, sizeof(double));
    double* lbu = lubu;
    double* ubu = lubu + NBU;
    ubu[0] = 5000;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
    }
    free(idxbu);
    free(lubu);








    // x
    int* idxbx = malloc(NBX * sizeof(int));
    idxbx[0] = 0;
    double* lubx = calloc(2*NBX, sizeof(double));
    double* lbx = lubx;
    double* ubx = lubx + NBX;
    lbx[0] = 290.15;
    ubx[0] = 298.15;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbx", idxbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubx", ubx);
    }
    free(idxbx);
    free(lubx);







    /* terminal constraints */













}


static void hvac_acados_create_set_opts(hvac_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    void *nlp_opts = capsule->nlp_opts;

    /************************************************
    *  opts
    ************************************************/


    int nlp_solver_exact_hessian = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess", &nlp_solver_exact_hessian);

    int exact_hess_dyn = true;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess_dyn", &exact_hess_dyn);

    int exact_hess_cost = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess_cost", &exact_hess_cost);

    int exact_hess_constr = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess_constr", &exact_hess_constr);

    int fixed_hess = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "fixed_hess", &fixed_hess);

    double globalization_fixed_step_length = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization_fixed_step_length", &globalization_fixed_step_length);




    int with_solution_sens_wrt_params = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_solution_sens_wrt_params", &with_solution_sens_wrt_params);

    int with_value_sens_wrt_params = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_value_sens_wrt_params", &with_value_sens_wrt_params);

    double solution_sens_qp_t_lam_min = 0.000000001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "solution_sens_qp_t_lam_min", &solution_sens_qp_t_lam_min);

    int globalization_full_step_dual = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "globalization_full_step_dual", &globalization_full_step_dual);

    double levenberg_marquardt = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */
    int qp_solver_cond_N;const int qp_solver_cond_N_ori = 576;
    qp_solver_cond_N = N < qp_solver_cond_N_ori ? N : qp_solver_cond_N_ori; // use the minimum value here
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    int nlp_solver_ext_qp_res = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "ext_qp_res", &nlp_solver_ext_qp_res);

    bool store_iterates = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "store_iterates", &store_iterates);
    int log_primal_step_norm = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "log_primal_step_norm", &log_primal_step_norm);

    double nlp_solver_tol_min_step_norm = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_min_step_norm", &nlp_solver_tol_min_step_norm);
    // set HPIPM mode: should be done before setting other QP solver options
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_hpipm_mode", "BALANCE");



    int qp_solver_t0_init = 2;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_t0_init", &qp_solver_t0_init);




    // set SQP specific options
    double nlp_solver_tol_stat = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_stat", &nlp_solver_tol_stat);

    double nlp_solver_tol_eq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_eq", &nlp_solver_tol_eq);

    double nlp_solver_tol_ineq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_ineq", &nlp_solver_tol_ineq);

    double nlp_solver_tol_comp = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_comp", &nlp_solver_tol_comp);

    int nlp_solver_max_iter = 100;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "max_iter", &nlp_solver_max_iter);

    // set options for adaptive Levenberg-Marquardt Update
    bool with_adaptive_levenberg_marquardt = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "with_adaptive_levenberg_marquardt", &with_adaptive_levenberg_marquardt);

    double adaptive_levenberg_marquardt_lam = 5;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_lam", &adaptive_levenberg_marquardt_lam);

    double adaptive_levenberg_marquardt_mu_min = 0.0000000000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_mu_min", &adaptive_levenberg_marquardt_mu_min);

    double adaptive_levenberg_marquardt_mu0 = 0.001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_mu0", &adaptive_levenberg_marquardt_mu0);

    bool eval_residual_at_max_iter = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "eval_residual_at_max_iter", &eval_residual_at_max_iter);

    int qp_solver_iter_max = 50;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_iter_max", &qp_solver_iter_max);



    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);
    int qp_solver_cond_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_ric_alg", &qp_solver_cond_ric_alg);

    int qp_solver_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_ric_alg", &qp_solver_ric_alg);


    int ext_cost_num_hess = 0;
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "cost_numerical_hessian", &ext_cost_num_hess);
    }
}


/**
 * Internal function for hvac_acados_create: step 7
 */
void hvac_acados_set_nlp_out(hvac_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;

    // initialize primal solution
    double* xu0 = calloc(NX+NU, sizeof(double));
    double* x0 = xu0;

    // initialize with x0
    x0[0] = 293.15;
    x0[1] = 293.15;
    x0[2] = 293.15;


    double* u0 = xu0 + NX;

    for (int i = 0; i < N; i++)
    {
        // x0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x0);
        // u0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x0);
    free(xu0);
}


/**
 * Internal function for hvac_acados_create: step 9
 */
int hvac_acados_create_precompute(hvac_solver_capsule* capsule) {
    int status = ocp_nlp_precompute(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    if (status != ACADOS_SUCCESS) {
        printf("\nocp_nlp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}


int hvac_acados_create_with_discretization(hvac_solver_capsule* capsule, int N, double* new_time_steps)
{
    // If N does not match the number of shooting intervals used for code generation, new_time_steps must be given.
    if (N != HVAC_N && !new_time_steps) {
        fprintf(stderr, "hvac_acados_create_with_discretization: new_time_steps is NULL " \
            "but the number of shooting intervals (= %d) differs from the number of " \
            "shooting intervals (= %d) during code generation! Please provide a new vector of time_stamps!\n", \
             N, HVAC_N);
        return 1;
    }

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    // 1) create and set nlp_solver_plan; create nlp_config
    capsule->nlp_solver_plan = ocp_nlp_plan_create(N);
    hvac_acados_create_set_plan(capsule->nlp_solver_plan, N);
    capsule->nlp_config = ocp_nlp_config_create(*capsule->nlp_solver_plan);

    // 2) create and set dimensions
    capsule->nlp_dims = hvac_acados_create_setup_dimensions(capsule);

    // 3) create and set nlp_opts
    capsule->nlp_opts = ocp_nlp_solver_opts_create(capsule->nlp_config, capsule->nlp_dims);
    hvac_acados_create_set_opts(capsule);

    // 4) create nlp_in
    capsule->nlp_in = ocp_nlp_in_create(capsule->nlp_config, capsule->nlp_dims);

    // 5) setup functions, nlp_in and default parameters
    hvac_acados_create_setup_functions(capsule);
    hvac_acados_setup_nlp_in(capsule, N, new_time_steps);
    hvac_acados_create_set_default_parameters(capsule);

    // 6) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts, capsule->nlp_in);

    // 7) create and set nlp_out
    // 7.1) nlp_out
    capsule->nlp_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    // 7.2) sens_out
    capsule->sens_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    hvac_acados_set_nlp_out(capsule);

    // 8) do precomputations
    int status = hvac_acados_create_precompute(capsule);

    return status;
}

/**
 * This function is for updating an already initialized solver with a different number of qp_cond_N. It is useful for code reuse after code export.
 */
int hvac_acados_update_qp_solver_cond_N(hvac_solver_capsule* capsule, int qp_solver_cond_N)
{
    // 1) destroy solver
    ocp_nlp_solver_destroy(capsule->nlp_solver);

    // 2) set new value for "qp_cond_N"
    const int N = capsule->nlp_solver_plan->N;
    if(qp_solver_cond_N > N)
        printf("Warning: qp_solver_cond_N = %d > N = %d\n", qp_solver_cond_N, N);
    ocp_nlp_solver_opts_set(capsule->nlp_config, capsule->nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    // 3) continue with the remaining steps from hvac_acados_create_with_discretization(...):
    // -> 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts, capsule->nlp_in);

    // -> 9) do precomputations
    int status = hvac_acados_create_precompute(capsule);
    return status;
}


int hvac_acados_reset(hvac_solver_capsule* capsule, int reset_qp_solver_mem)
{

    // set initialization to all zeros

    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;
    ocp_nlp_solver* nlp_solver = capsule->nlp_solver;

    double* buffer = calloc(NX+NU+NZ+2*NS+2*NSN+2*NS0+NBX+NBU+NG+NH+NPHI+NBX0+NBXN+NHN+NH0+NPHIN+NGN, sizeof(double));

    for(int i=0; i<N+1; i++)
    {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "sl", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "su", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "lam", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "z", buffer);
        if (i<N)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "pi", buffer);
        }
    }
    // get qp_status: if NaN -> reset memory
    int qp_status;
    ocp_nlp_get(capsule->nlp_solver, "qp_status", &qp_status);
    if (reset_qp_solver_mem || (qp_status == 3))
    {
        // printf("\nin reset qp_status %d -> resetting QP memory\n", qp_status);
        ocp_nlp_solver_reset_qp_memory(nlp_solver, nlp_in, nlp_out);
    }

    free(buffer);
    return 0;
}




int hvac_acados_update_params(hvac_solver_capsule* capsule, int stage, double *p, int np)
{
    int solver_status = 0;

    int casadi_np = 3;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }
    ocp_nlp_in_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, stage, "parameter_values", p);

    return solver_status;
}


int hvac_acados_update_params_sparse(hvac_solver_capsule * capsule, int stage, int *idx, double *p, int n_update)
{
    ocp_nlp_in_set_params_sparse(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, stage, idx, p, n_update);

    return 0;
}


int hvac_acados_set_p_global_and_precompute_dependencies(hvac_solver_capsule* capsule, double* data, int data_len)
{

    // printf("No global_data, hvac_acados_set_p_global_and_precompute_dependencies does nothing.\n");
    return 0;
}




int hvac_acados_solve(hvac_solver_capsule* capsule)
{
    // solve NLP
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}



int hvac_acados_setup_qp_matrices_and_factorize(hvac_solver_capsule* capsule)
{
    int solver_status = ocp_nlp_setup_qp_matrices_and_factorize(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}



void hvac_acados_batch_solve(hvac_solver_capsule ** capsules, int * status_out, int N_batch)
{

    for (int i = 0; i < N_batch; i++)
    {
        status_out[i] = ocp_nlp_solve(capsules[i]->nlp_solver, capsules[i]->nlp_in, capsules[i]->nlp_out);
    }


    return;
}


void hvac_acados_batch_setup_qp_matrices_and_factorize(hvac_solver_capsule ** capsules, int * status_out, int N_batch)
{

    for (int i = 0; i < N_batch; i++)
    {
        status_out[i] = ocp_nlp_setup_qp_matrices_and_factorize(capsules[i]->nlp_solver, capsules[i]->nlp_in, capsules[i]->nlp_out);
    }


    return;
}


void hvac_acados_batch_eval_params_jac(hvac_solver_capsule ** capsules, int N_batch)
{

    for (int i = 0; i < N_batch; i++)
    {
        ocp_nlp_eval_params_jac(capsules[i]->nlp_solver, capsules[i]->nlp_in, capsules[i]->nlp_out);
    }


    return;
}



void hvac_acados_batch_eval_solution_sens_adj_p(hvac_solver_capsule ** capsules, const char *field, int stage, double *out, int offset, int N_batch)
{


    for (int i = 0; i < N_batch; i++)
    {
        ocp_nlp_eval_solution_sens_adj_p(capsules[i]->nlp_solver, capsules[i]->nlp_in, capsules[i]->sens_out, field, stage, out + i*offset);
    }


    return;
}


void hvac_acados_batch_set_flat(hvac_solver_capsule ** capsules, const char *field, double *data, int N_data, int N_batch)
{
    int offset = ocp_nlp_dims_get_total_from_attr(capsules[0]->nlp_solver->config, capsules[0]->nlp_solver->dims, capsules[0]->nlp_out, field);

    if (N_batch*offset != N_data)
    {
        printf("batch_set_flat: wrong input dimension, expected %d, got %d\n", N_batch*offset, N_data);
        exit(1);
    }


    for (int i = 0; i < N_batch; i++)
    {
        ocp_nlp_set_all(capsules[i]->nlp_solver, capsules[i]->nlp_in, capsules[i]->nlp_out, field, data + i * offset);
    }


    return;
}



void hvac_acados_batch_get_flat(hvac_solver_capsule ** capsules, const char *field, double *data, int N_data, int N_batch)
{
    int offset = ocp_nlp_dims_get_total_from_attr(capsules[0]->nlp_solver->config, capsules[0]->nlp_solver->dims, capsules[0]->nlp_out, field);

    if (N_batch*offset != N_data)
    {
        printf("batch_get_flat: wrong input dimension, expected %d, got %d\n", N_batch*offset, N_data);
        exit(1);
    }


    for (int i = 0; i < N_batch; i++)
    {
        ocp_nlp_get_all(capsules[i]->nlp_solver, capsules[i]->nlp_in, capsules[i]->nlp_out, field, data + i * offset);
    }


    return;
}


int hvac_acados_free(hvac_solver_capsule* capsule)
{
    // before destroying, keep some info
    const int N = capsule->nlp_solver_plan->N;
    // free memory
    ocp_nlp_solver_opts_destroy(capsule->nlp_opts);
    ocp_nlp_in_destroy(capsule->nlp_in);
    ocp_nlp_out_destroy(capsule->nlp_out);
    ocp_nlp_out_destroy(capsule->sens_out);
    ocp_nlp_solver_destroy(capsule->nlp_solver);
    ocp_nlp_dims_destroy(capsule->nlp_dims);
    ocp_nlp_config_destroy(capsule->nlp_config);
    ocp_nlp_plan_destroy(capsule->nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < N; i++)
    {
        external_function_external_param_casadi_free(&capsule->discr_dyn_phi_fun[i]);
        external_function_external_param_casadi_free(&capsule->discr_dyn_phi_fun_jac_ut_xt[i]);
        
        
        external_function_external_param_casadi_free(&capsule->discr_dyn_phi_fun_jac_ut_xt_hess[i]);
    }
    free(capsule->discr_dyn_phi_fun);
    free(capsule->discr_dyn_phi_fun_jac_ut_xt);
  
  
    free(capsule->discr_dyn_phi_fun_jac_ut_xt_hess);

    // cost
    external_function_external_param_casadi_free(&capsule->ext_cost_0_fun);
    external_function_external_param_casadi_free(&capsule->ext_cost_0_fun_jac);
    external_function_external_param_casadi_free(&capsule->ext_cost_0_fun_jac_hess);
    
    
    for (int i = 0; i < N - 1; i++)
    {
        external_function_external_param_casadi_free(&capsule->ext_cost_fun[i]);
        external_function_external_param_casadi_free(&capsule->ext_cost_fun_jac[i]);
        external_function_external_param_casadi_free(&capsule->ext_cost_fun_jac_hess[i]);
        
        
    }
    free(capsule->ext_cost_fun);
    free(capsule->ext_cost_fun_jac);
    free(capsule->ext_cost_fun_jac_hess);

    // constraints



    return 0;
}


void hvac_acados_print_stats(hvac_solver_capsule* capsule)
{
    int nlp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_solver, "nlp_iter", &nlp_iter);
    ocp_nlp_get(capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_solver, "stat_m", &stat_m);


    double stat[1200];
    ocp_nlp_get(capsule->nlp_solver, "statistics", stat);

    int nrow = nlp_iter+1 < stat_m ? nlp_iter+1 : stat_m;


    printf("iter\tres_stat\tres_eq\t\tres_ineq\tres_comp\tqp_stat\tqp_iter\talpha");
    if (stat_n > 8)
        printf("\t\tqp_res_stat\tqp_res_eq\tqp_res_ineq\tqp_res_comp");
    printf("\n");
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < stat_n + 1; j++)
        {
            if (j == 0 || j == 5 || j == 6)
            {
                tmp_int = (int) stat[i + j * nrow];
                printf("%d\t", tmp_int);
            }
            else
            {
                printf("%e\t", stat[i + j * nrow]);
            }
        }
        printf("\n");
    }
}

int hvac_acados_custom_update(hvac_solver_capsule* capsule, double* data, int data_len)
{
    (void)capsule;
    (void)data;
    (void)data_len;
    printf("\ndummy function that can be called in between solver calls to update parameters or numerical data efficiently in C.\n");
    printf("nothing set yet..\n");
    return 1;

}



ocp_nlp_in *hvac_acados_get_nlp_in(hvac_solver_capsule* capsule) { return capsule->nlp_in; }
ocp_nlp_out *hvac_acados_get_nlp_out(hvac_solver_capsule* capsule) { return capsule->nlp_out; }
ocp_nlp_out *hvac_acados_get_sens_out(hvac_solver_capsule* capsule) { return capsule->sens_out; }
ocp_nlp_solver *hvac_acados_get_nlp_solver(hvac_solver_capsule* capsule) { return capsule->nlp_solver; }
ocp_nlp_config *hvac_acados_get_nlp_config(hvac_solver_capsule* capsule) { return capsule->nlp_config; }
void *hvac_acados_get_nlp_opts(hvac_solver_capsule* capsule) { return capsule->nlp_opts; }
ocp_nlp_dims *hvac_acados_get_nlp_dims(hvac_solver_capsule* capsule) { return capsule->nlp_dims; }
ocp_nlp_plan_t *hvac_acados_get_nlp_plan(hvac_solver_capsule* capsule) { return capsule->nlp_solver_plan; }
