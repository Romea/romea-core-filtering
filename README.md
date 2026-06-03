# romea_core_filtering

`romea_core_filtering` is a header-only C++ library that provides reusable building blocks for asynchronous data fusion.

The package does not define a complete application filter by itself. It provides the generic parts that are shared by many filters: timestamped state management, predictor and updater base classes, Gaussian and particle state representations, Kalman update equations, unscented transform utilities and particle resampling algorithms.

Application libraries, such as `romea_core_localisation`, derive from these base classes to define the state to estimate, the prediction model and the observation models used by their filters.

## 1) Filtering concept

The central class is `FilterBase<State, FSMState, Duration>`. It manages a time-ordered pool of filter states. Each stored element is a `FilterMetaState` containing:

* a timestamp;
* a state object;
* a finite-state-machine state;
* an update function associated with one observation.

When a new observation is inserted, `FilterBase` places it at the correct timestamp and recomputes the following stored states. Each recomputation alternates:

1. prediction from the previous timestamp to the current timestamp;
2. update with the observation attached to the current timestamp.

This is the mechanism that allows observations to arrive asynchronously and, within the retained state pool, slightly out of chronological order.

An illustration will be added here to show the time axis, the insertion of a delayed observation at its timestamp and the recomputation of the following states.

<!-- TODO: insert asynchronous_filtering_timeline image here. -->

## 2) Filters

The filtering layer contains the asynchronous prediction/update workflow and the filter families built on top of it.

### 2.1) Asynchronous filter base

The asynchronous filter base is the part that is common to all filtering families. It does not know whether the state is Gaussian, particle-based or represented in another way. Its role is to keep the timestamped state history consistent and to replay prediction/update steps when an observation is inserted.

| Component | Header | Role |
| --------- | ------ | ---- |
| `FilterBase` | `filter/filter_base.hpp` | Asynchronous filter base that stores timestamped states and applies prediction/update steps. |
| `FilterMetaState` | `filter/meta_state.hpp` | Timestamped state, finite-state-machine state and update callback. |
| `FilterPredictorBase` | `filter/predictor_base.hpp` | Partially defined prediction interface. Derived predictors provide the prediction model. |
| `FilterType` | `filter/type.hpp` | Enumeration used by client libraries to select a filtering family. |

`FilterBase` is independent of the mathematical representation of the state. The same asynchronous filtering logic can therefore be used with Gaussian states, particle states or other state representations, as long as the derived package provides compatible predictors and update functions.

### 2.2) Kalman filters

`KalmanFilter<State, FSMState, Duration>` implements the Kalman-filter variant of the asynchronous prediction/update workflow.

Kalman filters use Gaussian state representations and update the state through innovation, gain and covariance update equations. The package provides the generic equations and partially implemented updater base classes, while derived classes provide the observation model that is specific to the application.

The Kalman layer is organized as follows:

| Level | Headers | Role |
| ----- | ------- | ---- |
| Filter class | `filter/kalman/filter.hpp` | Provides the Kalman-filter specialization of the asynchronous filtering logic. |
| Updater base classes | `filter/kalman/updater/base/linear.hpp`, `extended.hpp`, `unscented.hpp` | Partially implemented updater base classes. They provide the common Kalman update flow while derived classes provide the observation model. |
| Equations | `filter/kalman/updater/equation/gain.hpp`, `innovation.hpp`, `mahalanobis.hpp`, `update.hpp` | Small reusable equations used by the updater base classes. |
| Traits | `filter/kalman/updater/traits.hpp` | Matrix type aliases used by Kalman updater equations. |

The available updater base classes are:

| Class | Purpose | Derived class must provide |
| ----- | ------- | -------------------------- |
| `LKFUpdaterBase` | Base for linear Kalman observation updates. | The innovation vector and the observation matrix. |
| `EKFUpdaterBase` | Alias of the linear updater base, used once the observation model has been linearized. | The nonlinear observation model evaluation, then the innovation vector and the linearized observation matrix. |
| `UKFUpdaterBase` | Base for unscented Kalman observation updates. | The propagation of each state sigma point through the observation model. |

The base classes are only partially implemented. They provide the generic update equations and the observation rejection mechanism based on Mahalanobis distance. Derived classes still have to compute the innovation, the observation matrix, or the propagated sigma-point observations depending on the filter type.

### 2.3) Particle filters

`ParticleFilter<State, FSMState, Duration>` implements the particle-filter variant of the asynchronous prediction/update workflow.

Particle filters represent the state with weighted particles. The package provides the particle state storage, updater base classes and resampling algorithms. Application-specific classes still define how each particle predicts an observation and how this observation should affect the particle weights.

The particle layer is organized as follows:

| Level | Headers | Role |
| ----- | ------- | ---- |
| Filter class | `filter/particle/filter.hpp` | Provides the particle-filter specialization of the asynchronous filtering logic. |
| State representation | `filter/particle/state.hpp` | Stores particles and their weights. |
| Updater base classes | `filter/particle/updater/base/generic.hpp`, `gaussian.hpp` | Partially implemented updater base classes. They provide the common particle update flow while derived classes provide the particle observation model. |
| Algorithms | `filter/particle/updater/algorithm/resampling.hpp` | Multinomial, stratified and systematic resampling algorithms. |

The available updater base classes are:

| Class | Purpose | Derived class must provide |
| ----- | ------- | -------------------------- |
| `PFUpdaterBase` | Common base for particle filter updaters. | The complete update logic for the particle observation model. |
| `PFGaussianUpdaterBase` | Base for particle filter updates driven by Gaussian observations. | The predicted observation associated with each particle. |

The base classes are only partially implemented. They provide the reusable weighting and resampling mechanisms. Derived classes still have to compute the predicted observation associated with each particle.

## 3) Mathematical tools

This package also provides mathematical tools that can be used independently from the asynchronous `FilterBase` logic.

### 3.1) Gaussian representations

| Component | Header | Role |
| --------- | ------ | ---- |
| `GaussianDistribution` | `gaussian/distribution.hpp` | First and second moments shared by Gaussian states and observations. |
| `GaussianState` | `gaussian/state.hpp` | Gaussian state vector and covariance accessors. |
| `GaussianInput` | `gaussian/input.hpp` | Gaussian input vector and covariance accessors. |
| `GaussianObservation` | `gaussian/observation.hpp` | Gaussian observation vector and covariance accessors. |

These types provide the common storage and accessors used by Kalman filters and by Gaussian observation models in particle filters.

### 3.2) Unscented transform

The unscented transform utilities are independent from `FilterBase` and can be reused anywhere a Gaussian distribution has to be converted to sigma points and back.

| Component | Header | Role |
| --------- | ------ | ---- |
| `UnscentedTransformParameters` | `unscented_transform/parameters.hpp` | Stores mean/covariance weights and scaling parameters. |
| `UnscentedTransformForward` | `unscented_transform/forward.hpp` | Converts a Gaussian distribution into sigma points. |
| `UnscentedTransformInverse` | `unscented_transform/inverse.hpp` | Reconstructs a Gaussian distribution from sigma points. |
| `UKFCorrelation` | `unscented_transform/correlation.hpp` | Computes state/observation correlation from sigma points. |

## 4) Minimal usage

A domain package usually defines:

* a state type, often derived from `GaussianState` or `ParticleFilterState`;
* a finite-state-machine state type;
* a predictor derived from `FilterPredictorBase`;
* one or more updater classes derived from the Kalman or particle updater base classes;
* result extraction helpers.

A complete filter is then assembled by creating the filter object, registering the predictor and inserting timestamped update functions:

```cpp
#include "romea_core_filtering/filter/kalman/filter.hpp"

using Duration = std::chrono::steady_clock::duration;

auto filter =
  std::make_unique<romea::core::KalmanFilter<State, FSMState, Duration>>(
    state_pool_size);

auto predictor = std::make_unique<MyPredictor>(...);
filter->register_predictor(std::move(predictor));

// ...

filter->process(observation_time, std::move(update_function));

// ...

State current_state;
filter->get_current_state(query_time, &current_state);
```

The filtering package provides the asynchronous filter base and the reusable equations. The application package provides `State`, `FSMState`, `MyPredictor` and the update functions.

## 5) Related packages

| Package | Role |
| ------- | ---- |
| `romea_core_localisation` | Localisation-specific states, predictors, updaters and result extraction helpers built on top of this package. |
| `romea_core_common` | Common mathematical and diagnostic utilities used by filtering clients. |

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

This library was written by **Jean Laneurit**, based on his thesis work under the supervision of **Roland Chapuis**, with scientific contributions from **Romuald Aufrere** and **Christophe Debain**.
