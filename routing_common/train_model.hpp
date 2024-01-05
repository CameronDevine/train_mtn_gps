#pragma once

#include "routing_common/vehicle_model.hpp"

namespace routing
{

class TrainModel : public VehicleModel
{
public:
  TrainModel();
  explicit TrainModel(LimitsInitList const & limits);
  TrainModel(VehicleModel::LimitsInitList const & limits, HighwayBasedSpeeds const & speeds);

  /// VehicleModelInterface overrides:
  SpeedKMpH GetTypeSpeed(feature::TypesHolder const & types, SpeedParams const & speedParams) const override;
  SpeedKMpH const & GetOffroadSpeed() const override;

  static TrainModel const & AllLimitsInstance();
};

class TrainModelFactory : public VehicleModelFactory
{
public:
  TrainModelFactory(CountryParentNameGetterFn const & countryParentNameGetterF);
};
}  // namespace routing
