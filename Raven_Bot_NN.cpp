#include "Raven_Bot_NN.h"

#include "Debug/DebugConsole.h"


Raven_Bot_NN::Raven_Bot_NN(Raven_Game* world, Vector2D pos) : Raven_Bot(world, pos)
{
	InitAccuracyLogic();
}


Raven_Bot_NN::~Raven_Bot_NN()
{
	fann_destroy(ann);
}

//---------------------------- Accuracy -----------------------------------
//
//-----------------------------------------------------------------------------
double Raven_Bot_NN::GetAccuracy()
{
	fann_type *calc_out;
	fann_type input[3];

	input[0] = Vec2DDistance(Pos(), GetTargetSys()->GetTarget()->Pos());
	input[1] = Velocity().Length();
	input[2] = GetTargetSys()->GetTimeTargetHasBeenVisible();

	calc_out = fann_run(ann, input);

	debug_con << "Accuracy NN returned " << calc_out[0] ;
	return calc_out[0];
}

//-------------------------  InitAccuracyLogicForAiming -------------------
//
//  set up some fuzzy variables and rules
//-----------------------------------------------------------------------------
void Raven_Bot_NN::InitAccuracyLogic() {
	ann = fann_create_standard(num_layers, num_input, num_neurons_hidden, num_output);

	fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
	fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC);
}