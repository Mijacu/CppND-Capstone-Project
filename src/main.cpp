/**
 * #include <iostream>
#include <vector>

#include "Position.h"
#include "RoboticArm.h"
#include "CircularTrajectory.h"

int main() {
    std::vector<double> links {10.0, 5.0};
    RoboticArm robotic_arm(links);

    Position position(1.0, 2.0, 4.0);
    position.print("position: ");

    arma::mat postion = robotic_arm.computePosition();
    position.print("Computed positions: ");

    std::vector<double> desired_position{0.0, 15.0};
    robotic_arm.computeAngles(desired_position);
    robotic_arm.printAngles();

    desired_position[0] = -15.0;
    desired_position[1] = 0.0;
    robotic_arm.computeAngles(desired_position);
    robotic_arm.printAngles();

    CircularTrajectory circular_trajectory;
    std::vector<std::vector<double>> trajectory = circular_trajectory.generateTrajectory();
    for (std::vector<double> position : trajectory) {
        robotic_arm.computeAngles(position);
        robotic_arm.printAngles();
    }

    return 0;
}


#include <cstdio>
#include <exception>

#include "CircularTrajectory.h"
#include "RoboticArm.h"
#include "Simbody.h"

using std::cout; using std::cin; using std::endl;

using namespace SimTK;

static int getNumFrames();
static int getNumObservations();
static Real getFrameTime(int i);
static const char* const* getObservationOrder();
static ArrayViewConst_<Vec3> getFrame(int i);
int main() {
    cout << "Hello" ;

  try
  { // Create the system.
    
    MultibodySystem         system;
    SimbodyMatterSubsystem  matter(system);
    GeneralForceSubsystem   forces(system);
    Force::UniformGravity   gravity(forces, matter, Vec3(0, -9.8, 0));

// BODY ground
Markers& markers = *new Markers();
DecorativeGeometry markerDecoration = DecorativeSphere(0.1).setColor(Blue);
MobilizedBody& mobod_ground = matter.updGround();
// END BODY ground


// Kludge to get the initial view large enough.
mobod_ground.addBodyDecoration(Vec3(-1,-1,-1), markerDecoration);
mobod_ground.addBodyDecoration(Vec3(1,1,1), markerDecoration);

// BODY loadcell1
Body::Rigid body_loadcell1(MassProperties(0,Vec3(0),Inertia(0,0,0)));
// Joint name is gnd_loadcell1
MobilizedBody::Free mobod_loadcell1(mobod_ground,Vec3(0), body_loadcell1,Vec3(0));
// END BODY loadcell1

// BODY loadcell2
Body::Rigid body_loadcell2(MassProperties(0,Vec3(0),Inertia(0,0,0)));
// Joint name is gnd_loadcell2
MobilizedBody::Free mobod_loadcell2(mobod_ground,Vec3(0), body_loadcell2,Vec3(0));
// END BODY loadcell2

// BODY base
Body::Rigid body_base(MassProperties(10,Vec3(0.0145,0.256438,0),Inertia(2.03337,0.281964,2.19416,-0.0900492,0,0)));
body_base.addDecoration(Vec3(0,.25,0), DecorativeBrick(Vec3(.1 ,.2 ,.3)).setColor(Green).setOpacity(.2));
// Joint name is base
MobilizedBody::Free mobod_base(mobod_ground, Vec3(0), body_base,Vec3(0));
// markers.addMarker("Base",mobod_base,Vec3(0),1);
// body_base.addDecoration(Vec3(0), markerDecoration);
// END BODY base

// BODY link_1
Body::Rigid body_link_1(MassProperties(10.5225,Vec3(0,-0.162162,0),Inertia(0.455313,0.036634,0.455313)));
body_link_1.addDecoration(Vec3(0,-.2,0), DecorativeEllipsoid(Vec3(.08,.2,.1)).setColor(Red));
// Joint name is joint_1
MobilizedBody::Ball mobod_link_1(mobod_base, Vec3(1, 1, 1), body_link_1,Vec3(0));
markers.addMarker("joint_1",mobod_link_1,Vec3(0,-0.404,0.05),1);
body_link_1.addDecoration(Vec3(0,-0.404,0.05),markerDecoration);
// END BODY link_1

// BODY link_2
Body::Rigid body_link_2(MassProperties(3.2475,Vec3(0,-0.171713,0),Inertia(0.125753,0.005157,0.126985)));
body_link_2.addDecoration(Vec3(0,-.2,0), DecorativeEllipsoid(Vec3(.05,.2,.07)).setColor(Yellow));
// Joint name is joint_2
MobilizedBody::Pin mobod_link_2(mobod_link_1,Vec3(0,-.4,0), body_link_2, Vec3(0));
markers.addMarker("joint_2",mobod_link_2,Vec3(-0.005,-0.41,0.053),1);
body_link_2.addDecoration(Vec3(-0.005,-0.41,0.053),markerDecoration);
// END BODY link_2

    matter.setShowDefaultGeometry(false);
    Visualizer viz(system);
    system.addEventReporter(new Visualizer::Reporter(viz, 0.1));


    // Initialize the system and state.
    
    system.realizeTopology();
    State state = system.getDefaultState();

    // Show initial configuration
    viz.report(state);
    State tempState = state; 
    matter.setUseEulerAngles(tempState, true);
    system.realizeModel(tempState);
    system.realize(tempState, Stage::Position);
    cout << "INITIAL CONFIGURATION\n"; 
    cout << tempState.getNU() << " dofs, " 
         << tempState.getNQErr() << " constraints.\n";
    
    cout << "Type any character to continue:\n";
    getchar();


    Assembler ik(system);
    ik.adoptAssemblyGoal(&markers);

    ik.lockMobilizer(mobod_base);

    markers.defineObservationOrder(getNumObservations(), getObservationOrder());
    markers.moveAllObservations(getFrame(0)); // initial observed locations
    state.setTime(getFrameTime(0));

    ik.initialize(state);
    printf("UNASSEMBLED CONFIGURATION (err=%g, cost=%g, qerr=%g)\n",
        ik.calcCurrentErrorNorm(), ik.calcCurrentGoal(),
        max(abs(ik.getInternalState().getQErr())));
    cout << "num freeQs: " << ik.getNumFreeQs() << endl;

    const Real Accuracy = 1e-3;
    ik.setAccuracy(Accuracy);
    ik.assemble(); // solve first frame.
    ik.updateFromInternalState(state);
    viz.report(state);
    printf("ASSEMBLED CONFIGURATION (acc=%g tol=%g err=%g, cost=%g, qerr=%g)\n",
        ik.getAccuracyInUse(), ik.getErrorToleranceInUse(), 
        ik.calcCurrentErrorNorm(), ik.calcCurrentGoal(),
        max(abs(ik.getInternalState().getQErr())));
    printf("# initializations=%d\n", ik.getNumInitializations());
    printf("# assembly steps: %d\n", ik.getNumAssemblySteps());
    printf(" evals: goal=%d grad=%d error=%d jac=%d\n",
        ik.getNumGoalEvals(), ik.getNumGoalGradientEvals(),
        ik.getNumErrorEvals(), ik.getNumErrorJacobianEvals());

    cout << "Type any character to continue:\n";
    getchar();

    const int NSteps = 1000;
    const int NToSkip = 4; // show every nth frame
    const double startReal = realTime(), startCPU = cpuTime();
    for (int f=1; f < NSteps; ++f) {
        markers.moveAllObservations(getFrame(f));
        // update internal state to match new observed locations
        ik.track(getFrameTime(f)); 
        if ((f%NToSkip)==0) {
            ik.updateFromInternalState(state);
            viz.report(state);
        }
    }

    cout << "ASSEMBLED " << NSteps-1 << " steps in " <<
        cpuTime()-startCPU << " CPU s, " << realTime()-startReal << " REAL s\n";

    printf("FINAL CONFIGURATION (acc=%g tol=%g err=%g, cost=%g, qerr=%g)\n",
        ik.getAccuracyInUse(), ik.getErrorToleranceInUse(), 
        ik.calcCurrentErrorNorm(), ik.calcCurrentGoal(),
        max(abs(ik.getInternalState().getQErr())));

    const double oons = Real(1)/ik.getNumAssemblySteps();
    printf("# initializations=%d\n", ik.getNumInitializations());
    printf("# assembly steps: %d\n", ik.getNumAssemblySteps());
    printf("  avg evals: goal=%g grad=%g error=%g jac=%g\n",
        ik.getNumGoalEvals()*oons, ik.getNumGoalGradientEvals()*oons,
        ik.getNumErrorEvals()*oons, ik.getNumErrorJacobianEvals()*oons);


    cout << "DONE ASSEMBLING\n";
    viz.report(state);

    cout << "Type any character to continue:\n";
    getchar();
   
  } catch (const std::exception& e) {
    std::printf("EXCEPTION THROWN: %s\n", e.what());
    exit(1);

  } catch (...) {
    std::printf("UNKNOWN EXCEPTION THROWN\n");
    exit(1);
  }

    return 0;
}

static const int NObservations = 2;
static const int NFrames = 1000;
static Real frameTimes[NFrames] = {0};
static const char* observationOrder[NObservations] = {"joint_1", "joint_2"};
static double observations [NFrames] [NObservations];
static int getNumFrames() {return NFrames;}
static int getNumObservations() {return NObservations;}
static Real getFrameTime(int i) {return frameTimes[i];}
static const char* const* getObservationOrder() {return observationOrder;}
static ArrayViewConst_<Vec3> getFrame(int i) {
    const Vec3* o = (const Vec3*)observations;
    return ArrayViewConst_<Vec3>(o+i*getNumObservations(), o+(i+1)*getNumObservations());
}

*/

/* -------------------------------------------------------------------------- *
 *                  Simbody(tm) Example: Custom Knee Joint                    *
 * -------------------------------------------------------------------------- *
 * This is part of the SimTK biosimulation toolkit originating from           *
 * Simbios, the NIH National Center for Physics-Based Simulation of           *
 * Biological Structures at Stanford, funded under the NIH Roadmap for        *
 * Medical Research, grant U54 GM072970. See https://simtk.org/home/simbody.  *
 *                                                                            *
 * Portions copyright (c) 2007-12 Stanford University and the Authors.        *
 * Authors: Ajay Seth                                                         *
 * Contributors: Michael Sherman                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


/*
 * This is a 2D knee joint example that demonstrates using custom mobilizers 
 * (FunctionBased) to simulate the effects of joint geometry that leads to the 
 * translation of tibia (shank) with respect to the femur (thigh) during flexion
 * and extension of the knee. 
 *
 * For more information see:
 *   A. Seth, M. Sherman, P. Eastman and S. Delp. Minimal formulation of joint 
 *   motion for biomechanisms. Nonlinear Dynamics, 2010. 
 *   DOI 10.1007/s11071-010-9717-3
 */

#include "Simbody.h"

using namespace SimTK;
using namespace std;

static const Real m = 1;   // kg
static const Real g = 9.8; // meters/s^2; apply in y direction
static const Real d = 0.4; // meters
static const Real Deg2Rad = Pi/180;

static const Real Accuracy = 1e-3;  // integration accuracy

// The real deviation of a knee from a pin joint is subtle; it is more fun to
// see if it is exaggerated, but the correct values here should be 1 to match real data.
static const Real ExaggerateX = 5;
static const Real ExaggerateY = 5;
#define SHOULD_EXAGGERATE // by default we'll take the more fun option; comment out for accuracy

//------------------------------------------------------------------------------
// We'll print out the total system energy to validate that it is conserved
// to within integrator tolerance (roughly), as it should be for this system since
// there is no energy input or loss. Try tightening the accuracy setting above to
// see whether energy conservation follows.
//------------------------------------------------------------------------------
class MyEnergyReporter : public PeriodicEventReporter {
public:
    MyEnergyReporter(const MultibodySystem& system, Real period) 
    : PeriodicEventReporter(period), system(system) {}

    virtual void handleEvent(const State& state) const override {
        cout << "t=" << state.getTime();
        cout << " E=" << system.calcEnergy(state) << endl;
    }
private:
    const MultibodySystem& system;
};

//------------------------------------------------------------------------------
// This force element is a crude stop that turns on a spring when the knee
// angle goes outside the range [low,high]. The spline data we have has a
// limited range that we can't exceed.
//------------------------------------------------------------------------------
class MyStop : public Force::Custom::Implementation {
public:
    MyStop(const MobilizedBody& knee, Real low, Real high, Real stiffness) 
    :   knee(knee), low(low), high(high), k(stiffness) 
    {   assert(low <= high && stiffness >= 0); }

    virtual void calcForce(const State& state, Vector_<SpatialVec>& bodyForces, 
                           Vector_<Vec3>& particleForces, Vector& mobilityForces) const override
    {
        const Real q = knee.getOneQ(state, 0);
        const Real x = q < low ? q-low : (q > high ? q-high : 0);
        knee.applyOneMobilityForce(state, 0, -k*x, mobilityForces);
    }

    virtual Real calcPotentialEnergy(const State& state) const override {
        const Real q = knee.getOneQ(state, 0);
        const Real x = q < low ? q-low : (q > high ? q-high : 0);
        return k*x*x/2;
    }

private:
    const MobilizedBody& knee;
    Real low, high, k;
};

//------------------------------------------------------------------------------
// main program
//------------------------------------------------------------------------------
int main(int argc, char** argv) {
try { // If anything goes wrong, an exception will be thrown.

    int i = 0;

    //--------------------------------------------------------------------------
    // Experimental data points (x,y) of tibia origin (tibial plateau) measured 
    // w.r.t. the origin of the femur (hip joint center) in the femur frame as a 
    // function of knee joint angle. From Yamaguchi and Zajac, 1989.
    //--------------------------------------------------------------------------
    // Tibia X:
    int npx = 12;
    Real angX[] = {-2.094395102393, -1.745329251994, -1.396263401595, -1.047197551197, 
                   -0.698131700798, -0.349065850399, -0.174532925199, 0.197344221443, 
                   0.337394955864, 0.490177570472, 1.521460267071, 2.094395102393};
    Real kneeX[] = {-0.003200000000, 0.001790000000, 0.004110000000, 0.004100000000, 
                    0.002120000000, -0.001000000000, -0.003100000000, -0.005227000000, 
                    -0.005435000000, -0.005574000000, -0.005435000000, -0.005250000000};
    // Tibia Y; note that Y data is offset by -0.4 due to body frame placement.
    int npy = 7;
    Real angY[] = {-2.094395102393, -1.221730476396, -0.523598775598, -0.349065850399, 
                   -0.174532925199, 0.159148563428, 2.094395102393};
    Real kneeY[] = {-0.422600000000, -0.408200000000, -0.399000000000, -0.397600000000, 
                    -0.396600000000, -0.395264000000, -0.396000000000 };

    // Create SimTK Vectors to hold data points, and initialize from above arrays.
    Vector ka_x (npx, angX); // measured knee angles when X data was collected
    Vector ka_y (npy, angY); // measured knee angles when Y data was collected
    Vector tib_x(npx, kneeX);
    Vector tib_y(npy, kneeY);

    #ifdef SHOULD_EXAGGERATE
        // See above.
        tib_x *= ExaggerateX;
        tib_y = (tib_y+0.4)*ExaggerateY - 0.4; // exaggerate deviation only, not offset
    #endif

    // Generate splines from vectors of data points.
    const int Degree = 3; // use cubics
    SplineFitter<Real> fitterX = SplineFitter<Real>::fitFromGCV(Degree, ka_x, tib_x);
    SplineFitter<Real> fitterY = SplineFitter<Real>::fitFromGCV(Degree, ka_y, tib_y);
    Spline fx = fitterX.getSpline();
    Spline fy = fitterY.getSpline();

    //--------------------------------------------------------------------------
       // Define the 6-spatial functions that specify the motion of the tibia as a 
    // a FunctionBased MobilizedBody (shank) w.r.t. its parent (the thigh). 
    //--------------------------------------------------------------------------
    // Each function has to return 1 Real value
    std::vector<const Function*> functions(6);
    // as a function of coordIndices (more than one per function) 
    std::vector< std::vector<int> > coordIndices(6);
    // about a body-fixed axis for rotations or in parent translations 
    std::vector<Vec3> axes(6);

    // Set the 1st and 2nd spatial rotation about the orthogonal (X then Y) axes as 
    // constant values. That is they don't contribute to motion nor do they have   
    // any coordinates in the equations of motion.
    // |--------------------------------|
    // | 1st function: rotation about X |
    // |--------------------------------|
    functions[0] = (new Function::Constant(0, 0));
    std::vector<int> noIndex(0);
    coordIndices[0] =(noIndex);

    // |--------------------------------|
    // | 2nd function: rotation about Y |
    // |--------------------------------|
    functions[1] = (new Function::Constant(0, 0));
    coordIndices[1] = (noIndex);

    // Set the spatial rotation about third axis to be the knee-extension
    // angle (the one q) about the Z-axis of the tibia at the femoral condyles
    // Define the coefficients of the linear function of the knee-angle with the
    // spatial rotation about Z.
    Vector coeff(2);
    // Linear function x3 = coeff[0]*q + coeff[1]
    coeff[0] = 1;  coeff[1] = 0;
    // |--------------------------------|
    // | 3rd function: rotation about Z |
    // |--------------------------------|
    functions[2] = new Function::Linear(coeff);
    // function of coordinate 0 (knee extension angle)
    std::vector<int> qIndex(1,0);
    coordIndices[2] = qIndex;

    // Set the spatial translations as a function (splines) along the parent X and Y axes
    // |-----------------------------------|
    // | 4th function: translation about X |
    // |-----------------------------------|
    functions[3] = new Spline(fx); // Give the mobilizer a copy it can own.
    coordIndices[3] =(qIndex);

    // |-----------------------------------|
    // | 5th function: translation about Y |
    // |-----------------------------------|
    functions[4] = new Spline(fy); // Give the mobilizer a copy it can own.
    coordIndices[4] =(qIndex);

    // |-----------------------------------|
    // | 6th function: translation about Z |
    // |-----------------------------------|
    functions[5] = (new Function::Constant(0, 0));
    coordIndices[5] = (noIndex);

    // Construct the multibody system
    const Real grav = 9.80665;
    MultibodySystem system; system.setUseUniformBackground(true);
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    Force::Gravity gravity(forces, matter, -YAxis, grav);
    matter.setShowDefaultGeometry(true);

    //--------------------------------------------------------------------------
    // Define the model's physical (body) properties
    //--------------------------------------------------------------------------
    //Thigh
    Body::Rigid femur(MassProperties(8.806, Vec3(0), Inertia(Vec3(0.1268, 0.0332, 0.1337))));
    femur.addDecoration(Transform(Vec3(0, -0.21+0.1715, 0)), 
        DecorativeCylinder(0.04, 0.21).setColor(Orange).setOpacity(.5));

    //Shank
    Body::Rigid tibia(MassProperties(3.510, Vec3(0), Inertia(Vec3(0.0477, 0.0048, 0.0484))));
    tibia.addDecoration(Transform(Vec3(0, -0.235+0.1862, 0)), 
        DecorativeCylinder(0.035, 0.235).setColor(Red));

    //--------------------------------------------------------------------------
    // Build the multibody system by adding mobilized bodies to the matter subsystem
    //--------------------------------------------------------------------------
    // Add the thigh via hip joint
    MobilizedBody::Pin thigh(matter.Ground(), Transform(Vec3(0)), femur, Transform(Vec3(0.0020, 0.1715, 0)));

    // This is how you might model the knee if you could only use a pin joint.
    //MobilizedBody::Pin shank(thigh, Transform(Vec3(0.0033, -0.2294, 0)), 
    //                         tibia, Transform(Vec3(0.0, 0.1862, 0.0)));
    
    // NOTE: function of Y-translation data was defined int the femur frame 
    // according to Yamaguchi and Zajac, which had the orgin at the hip joint 
    // center and the Y along the long-axis of the femur and Z out of the page. 
    MobilizedBody::FunctionBased shank(thigh, Transform(Vec3(0.0020, 0.1715, 0)), 
                                       tibia, Transform(Vec3(0.0, 0.1862, 0.0)), 
                                       1, // # of mobilities (dofs) for this joint
                                       functions, coordIndices);

    // Add some stop springs so the knee angle won't get outside the range of spline 
    // data we have. This custom force element is defined above.
    Force::Custom(forces, new MyStop(shank, -Pi/2, 0*Pi, 100));


    //--------------------------------------------------------------------------
    // Setup reporters so we can get some output.
    //--------------------------------------------------------------------------
    // Vizualizer Animation
    Visualizer viz(system);
    system.addEventReporter(new Visualizer::Reporter(viz, 0.01));
    // Energy -- reporter defined above.
    system.addEventReporter(new MyEnergyReporter(system, 0.01));
    
    //--------------------------------------------------------------------------
    // Complete the construction of the "const" part of the System and
    // allocate the default state.
    //--------------------------------------------------------------------------
    system.realizeTopology();
    // Get a copy of the default state to work with.
    State state = system.getDefaultState();

    //--------------------------------------------------------------------------
    // Set modeling options if any (this one is not actually needed here).
    //--------------------------------------------------------------------------
    matter.setUseEulerAngles(state, true);
    // Complete construction of the model, allocating additional state variables
    // if necessary.
    system.realizeModel(state);

    //--------------------------------------------------------------------------
    // Set initial conditions.
    //--------------------------------------------------------------------------
    // Hip and knee coordinates and speeds similar to early swing
    double hip_angle = -45*Pi/180;
    double knee_angle = 0*Pi/180;
    double hip_vel = 1;
    double knee_vel = -5.0;

    // Set initial states (Q's and U's)
    // Position
    thigh.setOneQ(state, 0, hip_angle);
    shank.setOneQ(state, 0, knee_angle);
    // Speed
    thigh.setOneU(state, 0, hip_vel);
    shank.setOneU(state, 0, knee_vel);
    
    //--------------------------------------------------------------------------
    // Run simulation.
    //--------------------------------------------------------------------------
    RungeKuttaMersonIntegrator integ(system);
    integ.setAccuracy(Accuracy);
    TimeStepper ts(system, integ);
    ts.initialize(state); // set IC's
    ts.stepTo(5.0);
} 
catch (const exception& e) {
    printf("EXCEPTION THROWN: %s\n", e.what());
    exit(1);
}

    return 0;
}

