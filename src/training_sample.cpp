#include "training_sample.h"

TrainingSample::TrainingSample(bool isPositive) :
  cloud (new PointCloudT),
  isPositive (isPositive)
{}

TrainingSample::~TrainingSample() {}
