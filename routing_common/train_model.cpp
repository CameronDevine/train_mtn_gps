#include "routing_common/train_model.hpp"

#include "indexer/classificator.hpp"

namespace train_model
{
using namespace routing;

// See model specifics in different countries here:
//   https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Access-Restrictions

// See road types here:
//   https://wiki.openstreetmap.org/wiki/Key:highway

// |kSpeedOffroadKMpH| is a speed which is used for edges that don't lie on road features.
// For example for pure fake edges. In car routing, off road speed for calculation ETA is not used.
// The weight of such edges is considered as 0 seconds. It's especially actual when an airport is
// a start or finish. On the other hand, while route calculation the fake edges are considered
// as quite heavy. The idea behind that is to use the closest edge for the start and the finish
// of the route except for some edge cases.
SpeedKMpH constexpr kSpeedOffroadKMpH = {0.01 /* weight */, kNotUsed /* eta */};

HighwayBasedFactors const kDefaultFactors = {
    {HighwayType::MiniatureRailway, InOutCityFactor(1.0)},
    {HighwayType::MiniatureRailwayYard, InOutCityFactor(1.0)},
    {HighwayType::MiniatureRailwayMain, InOutCityFactor(1.0)},
};

VehicleModel::LimitsInitList const kDefaultOptions = {
    {HighwayType::MiniatureRailway, true},
    {HighwayType::MiniatureRailwayMain, true},
    {HighwayType::MiniatureRailwayYard, true}
};

HighwayBasedSpeeds const kDefaultSpeeds = {
    {HighwayType::MiniatureRailway, InOutCitySpeedKMpH(SpeedKMpH(8.0))},
    {HighwayType::MiniatureRailwayYard, InOutCitySpeedKMpH(SpeedKMpH(4.8))},
    {HighwayType::MiniatureRailwayMain, InOutCitySpeedKMpH(SpeedKMpH(11.2))},
};

VehicleModel::SurfaceInitList const kTrainSurface = {};

} // End train_model namespace

namespace routing
{
TrainModel::TrainModel() : TrainModel(train_model::kDefaultOptions)
{
}

TrainModel::TrainModel(VehicleModel::LimitsInitList const & limits)
  : TrainModel(limits, train_model::kDefaultSpeeds)
{
}

TrainModel::TrainModel(VehicleModel::LimitsInitList const & limits, HighwayBasedSpeeds const & speeds)
  : VehicleModel(classif(), limits, train_model::kTrainSurface, {speeds, train_model::kDefaultFactors})
{
  using namespace train_model;

  ASSERT_EQUAL(kDefaultOptions.size(), kDefaultSpeeds.size(), ());

  SpeedKMpH constexpr kMaxTrainSpeedKMpH(16);
  CHECK_LESS(m_maxModelSpeed, kMaxTrainSpeedKMpH, ());
  m_maxModelSpeed = kMaxTrainSpeedKMpH;
}


SpeedKMpH TrainModel::GetTypeSpeed(feature::TypesHolder const & types, SpeedParams const & speedParams) const
{
  return GetTypeSpeedImpl(types, speedParams, true /* isCar */);
}

SpeedKMpH const & TrainModel::GetOffroadSpeed() const { return train_model::kSpeedOffroadKMpH; }

// static
TrainModel const & TrainModel::AllLimitsInstance()
{
  static TrainModel const instance;
  return instance;
}

TrainModelFactory::TrainModelFactory(CountryParentNameGetterFn const & countryParentNameGetterFn)
  : VehicleModelFactory(countryParentNameGetterFn)
{
  using namespace train_model;
  using std::make_shared;

  m_models[""] = make_shared<TrainModel>();
}
}  // namespace routing
