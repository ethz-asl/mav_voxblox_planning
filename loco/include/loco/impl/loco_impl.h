#ifndef LOCO_LOCO_IMPL_LOCO_IMPL_H_
#define LOCO_LOCO_IMPL_LOCO_IMPL_H_

namespace loco {

template <int N = 10>
Loco::Loco(size_t dimension) : Loco(dimension, Config()) {}

Loco::Loco(size_t dimension, const Config& config)
    : poly_opt_(dimension),
      K_(dimension),
      num_free_(0) {}

      : poly_opt_(dimension),
        K_(dimension),
        epsilon_(0.5),
        bounding_sphere_radius_(0.5),
        w_d_(0.1),
        w_c_(10),
        w_g_(2.5),
        collision_sampling_dt_(0.1),
        map_resolution_(0.1),
        verbose_(false),
        soft_goal_constraint_(true),
        num_free_(0) {}

}  // namespace loco

#endif LOCO_LOCO_IMPL_LOCO_IMPL_H_
