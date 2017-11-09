#pragma once
#include "Raven_Bot.h"
#include "doublefann.h"

class Raven_Bot_NN :
	public Raven_Bot
{
private:
	const unsigned int num_input = 3;
	const unsigned int num_output = 1;
	const unsigned int num_layers = 3;
	const unsigned int num_neurons_hidden = 3;
	const float desired_error = (const float) 0.001;
	const unsigned int max_epochs = 500000;
	const unsigned int epochs_between_reports = 1000;

	struct fann *ann;

public:
	Raven_Bot_NN(Raven_Game* world, Vector2D pos);
	~Raven_Bot_NN();
	
	virtual double GetAccuracy();
	void InitAccuracyLogic();
};

