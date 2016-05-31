//// MatPower constants... ////

///////////////////////////////////////////////////////////
// from MATPOWER file idx_bus
//
// bus types
//#define PQ       1
//#define PV       2
//#define REF      3
//#define NONE     4
// bus indeces
#define BUS_I        1    /* bus number (1 to 29997) */
#define BUS_TYPE     2    /* bus type (1 - PQ bus, 2 - PV bus, 3 - reference bus, 4 - isolated bus) */
#define PD           3    /* Pd, real power demand (MW) */
#define QD           4    /* Qd, reactive power demand (MVAr) */
#define GS           5    /* Gs, shunt conductance (MW at V  1.0 p.u.) */
#define BS           6    /* Bs, shunt susceptance (MVAr at V  1.0 p.u.) */
#define BUS_AREA     7    /* area number, 1-100 */
#define VM           8    /* Vm, voltage magnitude (p.u.) */
#define VA           9    /* Va, voltage angle (degrees) */
#define BASE_KV      10   /* baseKV, base voltage (kV) */
#define ZONE         11   /* zone, loss zone (1-999) */
#define VMAX         12   /* maxVm, maximum voltage magnitude (p.u.)      (not in PTI format) */
#define VMIN         13   /* minVm, minimum voltage magnitude (p.u.)      (not in PTI format) */
// included in opf solution, not necessarily in input
// assume objective function has units, u
#define LAM_P        14   /* Lagrange multiplier on real power mismatch (u/MW)       */
#define LAM_Q        15   /* Lagrange multiplier on reactive power mismatch (u/MVAr) */
#define MU_VMAX      16   /* Kuhn-Tucker multiplier on upper voltage limit (u/p.u.)  */
#define MU_VMIN      17   /* Kuhn-Tucker multiplier on lower voltage limit (u/p.u.)  */

///////////////////////////////////////////////////////////
// branch indeces from MATPOWER file idx_brch
//
#define F_BUS        1    /* f, from bus number   */
#define T_BUS        2    /* t, to bus number     */
#define BR_R         3    /* r, resistance (p.u.) */
#define BR_X         4    /* x, reactance (p.u.)  */
#define BR_B         5    /* b, total line charging susceptance (p.u.)  */
#define RATE_A       6    /* rateA, MVA rating A (long term rating)     */
#define RATE_B       7    /* rateB, MVA rating B (short term rating)    */
#define RATE_C       8    /* rateC, MVA rating C (emergency rating)     */
#define TAP          9    /* ratio, transformer off nominal turns ratio */
#define SHIFT        10   /* angle, transformer phase shift angle (degrees) */
#define BR_STATUS    11   /* initial branch status, 1 - in service, 0 - out of service */
// included in power flow solution, not necessarily in input
#define PF           12   /* real power injected at "from" bus end (MW)       (not in PTI format) */
#define QF           13   /* reactive power injected at "from" bus end (MVAr) (not in PTI format) */
#define PT           14   /* real power injected at "to" bus end (MW)         (not in PTI format) */
#define QT           15   /* reactive power injected at "to" bus end (MVAr)   (not in PTI format) */
// included in opf solution, not necessarily in input
// assume objective function has units, u
#define MU_SF        16   /* Kuhn-Tucker multiplier on MVA limit at "from" bus (u/MVA) */
#define MU_ST        17   /* Kuhn-Tucker multiplier on MVA limit at "to" bus (u/MVA) */

///////////////////////////////////////////////////////////
// gen constants from MATPOWER file idx_gen
#define GEN_BUS      1    /* bus number  */
#define PG           2    /* Pg, real power output (MW)       */
#define QG           3    /* Qg, reactive power output (MVAr) */
#define QMAX         4    /* Qmax, maximum reactive power output (MVAr)  */
#define QMIN         5    /* Qmin, minimum reactive power output (MVAr)  */
#define VG           6    /* Vg, voltage magnitude setpoint (p.u.)       */
#define MBASE        7    /* mBase, total MVA base of this machine, defaults to baseMVA  */
#define GEN_STATUS   8    /* status, 1 - machine in service, 0 - machine out of service  */
#define PMAX         9    /* Pmax, maximum real power output (MW) */
#define PMIN         10   /* Pmin, minimum real power output (MW) */
// included in opf solution, not necessarily in input
// assume objective function has units, u
#define MU_PMAX      11   /* Kuhn-Tucker multiplier on upper Pg limit (u/MW) */
#define MU_PMIN      12   /* Kuhn-Tucker multiplier on lower Pg limit (u/MW) */
#define MU_QMAX      13   /* Kuhn-Tucker multiplier on upper Qg limit (u/MVAr) */
#define MU_QMIN      14   /* Kuhn-Tucker multiplier on lower Qg limit (u/MVAr) */

///////////////////////////////////////////////////////////
// load constants as defined by me
#define L_BUS     1  /* bus number */
#define L_VALUE   2  /* load value */
#define L_PD      3  /* load real power (MW) */
#define L_QD      4  /* load reactive power (MVAR) */
#define L_GS      5  /* load shunt susceptance */
#define L_BS      6  /* load shunt reactance */
#define L_IRE     7  /* load real current */
#define L_IIM     8  /* load imag current */
#define L_STATUS  9  /* load status */
#define L_PMAX    10 /* load max P demand */
#define L_QMAX    11 /* load max Q demand */


