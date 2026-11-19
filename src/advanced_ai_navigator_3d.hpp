#pragma once

#include <godot_cpp/classes/ref.hpp>
#include <cstdint>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/navigation_path_query_parameters3d.hpp>
#include <godot_cpp/classes/navigation_path_query_result3d.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/rid.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/typed_array.hpp>
#include <godot_cpp/variant/array.hpp>

namespace godot {

class CharacterBody3D;

class AdvancedAINavigator3D : public Node3D {
	GDCLASS(AdvancedAINavigator3D, Node3D)

public:
	enum SteeringMode {
		STEERING_OMNI = 0,     // Agile NPCs / strafing units
		STEERING_VEHICLE = 1,  // Car / tank / forward-heading AI
	};

private:
	// Target & Pathing
	Vector3 target_position = Vector3(0.0f, 0.0f, 0.0f);
	bool navigation_active = false;
	uint32_t navigation_layers = 1;
	float waypoint_reached_distance = 1.0f;
	float target_reached_distance = 1.5f;
	float repath_interval = 0.25f;
	float repath_target_threshold = 0.75f;
	bool simplify_path = true;
	float simplify_epsilon = 0.2f;

	// Movement & Vehicle Hints
	SteeringMode steering_mode = STEERING_OMNI;
	bool auto_move_parent_body = true;
	float max_speed = 8.0f;
	float acceleration = 18.0f;
	float turn_speed = 8.0f;
	float gravity = 9.8f;

	// Server RVO Avoidance
	bool avoidance_enabled = true;
	float agent_radius = 0.6f;
	float agent_height = 1.8f;
	float neighbor_distance = 12.0f;
	int32_t max_neighbors = 8;
	float time_horizon_agents = 1.2f;
	float time_horizon_obstacles = 0.6f;
	uint32_t avoidance_layers = 1;
	uint32_t avoidance_mask = 1;

	// Context-Based Dynamic Obstacle Whiskers (for crashed cars / debris)
	bool local_whiskers_enabled = true;
	uint32_t obstacle_collision_mask = 1;
	float whisker_length = 4.5f;
	float whisker_spread_deg = 55.0f;
	float whisker_height_offset = 0.6f;
	float whisker_avoid_weight = 1.4f;

	// Stuck Recovery
	float stuck_check_time = 1.5f;
	float stuck_min_progress = 0.35f;

	// Internal Runtime State
	RID custom_map_rid;
	RID agent_rid;
	RID assigned_map_rid;

	Ref<NavigationPathQueryParameters3D> query_params;
	Ref<NavigationPathQueryResult3D> query_result;

	PackedVector3Array current_path;
	int32_t current_waypoint_idx = 0;
	int32_t last_notified_link_idx = -1;
	Vector3 last_queried_target = Vector3(0.0f, 0.0f, 0.0f);
	double repath_timer = 0.0;

	Vector3 current_velocity = Vector3(0.0f, 0.0f, 0.0f);
	Vector3 desired_velocity = Vector3(0.0f, 0.0f, 0.0f);
	Vector3 rvo_safe_velocity = Vector3(0.0f, 0.0f, 0.0f);
	bool has_rvo_velocity = false;

	// Vehicle / Physics Outputs
	float steer_hint = 0.0f;     // -1.0 (right) to +1.0 (left) relative to -Z forward
	float throttle_hint = 0.0f;  // -1.0 (reverse) to +1.0 (full throttle)

	// Stuck Tracking
	Vector3 stuck_anchor_pos = Vector3(0.0f, 0.0f, 0.0f);
	double stuck_timer = 0.0;
	double recovery_reverse_timer = 0.0;
	int32_t recovery_dir_sign = 1;

	// Smoothing & pacing
	float steer_smoothing = 8.0f;
	float arrival_slow_radius = 3.0f;
	float corner_slowdown = 0.6f;
	Vector3 smoothed_dir = Vector3(0.0f, 0.0f, 0.0f);
	Array whisker_debug;

	// Raycast budget
	int32_t whisker_update_interval = 2;
	float whisker_far_distance = 45.0f;
	Vector3 whisker_push = Vector3(0.0f, 0.0f, 0.0f);
	float whisker_min_ratio = 1.0f;
	bool whisker_cache_valid = false;
	uint32_t whisker_frame_counter = 0;
	uint32_t whisker_phase = 0;

	// Look-ahead (vehicle mode)
	float look_ahead_distance = 3.0f;
	float look_ahead_corner_limit_deg = 40.0f;

	// Unreachable targets
	float unreachable_tolerance = 1.5f;
	bool path_target_reachable = true;
	bool path_failed_notified = false;
	Vector3 last_failed_target = Vector3(0.0f, 0.0f, 0.0f);

	void _create_server_agent();
	void _free_server_agent();
	void _sync_agent_properties();
	void _on_avoidance_velocity_computed(const Vector3 &p_safe_velocity);

	RID _get_active_map_rid() const;
	Vector3 _compute_whisker_steering(const Vector3 &p_desired_dir, const TypedArray<RID> &p_exclude);
	void _update_vehicle_hints(const Vector3 &p_move_dir, float p_speed_ratio);
	void _update_stuck_detector(double p_delta, const Vector3 &p_pos);
	void _cast_whiskers(const Vector3 &p_flat_dir, const TypedArray<RID> &p_exclude);
	Vector3 _get_look_ahead_point(const Vector3 &p_pos) const;
	void _notify_path_failed(float p_gap);

protected:
	static void _bind_methods();
	void _notification(int p_what);

public:
	AdvancedAINavigator3D();
	~AdvancedAINavigator3D() override;

	void _ready() override;
	void _physics_process(double p_delta) override;

	// Path Control API
	void set_target_position(const Vector3 &p_target);
	Vector3 get_target_position() const { return target_position; }
	bool force_repath();
	void stop_navigation();
	bool is_navigation_finished() const;

	// Inspection & Vehicle Physics Integration API
	PackedVector3Array get_current_path() const { return current_path; }
	Vector3 get_next_waypoint() const;
	Vector3 get_desired_velocity() const { return desired_velocity; }
	Vector3 get_safe_velocity() const { return rvo_safe_velocity; }
	float get_steer_hint() const { return steer_hint; }
	float get_throttle_hint() const { return throttle_hint; }
	float get_remaining_distance() const;

	void set_custom_navigation_map(const RID &p_map);
	RID get_custom_navigation_map() const { return custom_map_rid; }

	// Property Getters & Setters

	void set_steering_mode(int p_mode) { steering_mode = (p_mode == STEERING_VEHICLE) ? STEERING_VEHICLE : STEERING_OMNI;}

	int get_steering_mode() const { return static_cast<int>(steering_mode); }

	void set_auto_move_parent_body(bool p_enable) { auto_move_parent_body = p_enable; }
	bool is_auto_move_parent_body() const { return auto_move_parent_body; }

	void set_navigation_layers(uint32_t p_layers) { navigation_layers = p_layers; }
	uint32_t get_navigation_layers() const { return navigation_layers; }

	void set_waypoint_reached_distance(float p_dist) { waypoint_reached_distance = p_dist; }
	float get_waypoint_reached_distance() const { return waypoint_reached_distance; }

	void set_target_reached_distance(float p_dist) { target_reached_distance = p_dist; }
	float get_target_reached_distance() const { return target_reached_distance; }

	void set_repath_interval(float p_sec) { repath_interval = p_sec; }
	float get_repath_interval() const { return repath_interval; }

	void set_simplify_path(bool p_enable) { simplify_path = p_enable; }
	bool is_simplify_path() const { return simplify_path; }

	void set_simplify_epsilon(float p_eps) { simplify_epsilon = p_eps; }
	float get_simplify_epsilon() const { return simplify_epsilon; }

	void set_max_speed(float p_speed);
	float get_max_speed() const { return max_speed; }

	void set_acceleration(float p_accel) { acceleration = p_accel; }
	float get_acceleration() const { return acceleration; }

	void set_turn_speed(float p_turn) { turn_speed = p_turn; }
	float get_turn_speed() const { return turn_speed; }

	void set_gravity(float p_grav) { gravity = p_grav; }
	float get_gravity() const { return gravity; }

	void set_avoidance_enabled(bool p_enable);
	bool is_avoidance_enabled() const { return avoidance_enabled; }

	void set_agent_radius(float p_radius);
	float get_agent_radius() const { return agent_radius; }

	void set_agent_height(float p_height);
	float get_agent_height() const { return agent_height; }

	void set_neighbor_distance(float p_dist);
	float get_neighbor_distance() const { return neighbor_distance; }

	void set_max_neighbors(int32_t p_max);
	int32_t get_max_neighbors() const { return max_neighbors; }

	void set_avoidance_layers(uint32_t p_layers);
	uint32_t get_avoidance_layers() const { return avoidance_layers; }

	void set_avoidance_mask(uint32_t p_mask);
	uint32_t get_avoidance_mask() const { return avoidance_mask; }

	void set_local_whiskers_enabled(bool p_enable) { local_whiskers_enabled = p_enable; }
	bool is_local_whiskers_enabled() const { return local_whiskers_enabled; }

	void set_obstacle_collision_mask(uint32_t p_mask) { obstacle_collision_mask = p_mask; }
	uint32_t get_obstacle_collision_mask() const { return obstacle_collision_mask; }

	void set_whisker_length(float p_len) { whisker_length = p_len; }
	float get_whisker_length() const { return whisker_length; }

	void set_whisker_spread_deg(float p_deg) { whisker_spread_deg = p_deg; }
	float get_whisker_spread_deg() const { return whisker_spread_deg; }

	void set_whisker_avoid_weight(float p_weight) { whisker_avoid_weight = p_weight; }
	float get_whisker_avoid_weight() const { return whisker_avoid_weight; }

	void set_steer_smoothing(float p_value) { steer_smoothing = p_value; }
	float get_steer_smoothing() const { return steer_smoothing; }
	void set_arrival_slow_radius(float p_value) { arrival_slow_radius = p_value; }
	float get_arrival_slow_radius() const { return arrival_slow_radius; }
	void set_corner_slowdown(float p_value) { corner_slowdown = p_value; }
	float get_corner_slowdown() const { return corner_slowdown; }
	Array get_whisker_debug() const { return whisker_debug; }
	void set_whisker_update_interval(int32_t p_frames) { whisker_update_interval = p_frames; }
	int32_t get_whisker_update_interval() const { return whisker_update_interval; }
	void set_whisker_far_distance(float p_dist) { whisker_far_distance = p_dist; }
	float get_whisker_far_distance() const { return whisker_far_distance; }
	void set_look_ahead_distance(float p_dist) { look_ahead_distance = p_dist; }
	float get_look_ahead_distance() const { return look_ahead_distance; }
	void set_look_ahead_corner_limit_deg(float p_deg) { look_ahead_corner_limit_deg = p_deg; }
	float get_look_ahead_corner_limit_deg() const { return look_ahead_corner_limit_deg; }
	void set_unreachable_tolerance(float p_dist) { unreachable_tolerance = p_dist; }
	float get_unreachable_tolerance() const { return unreachable_tolerance; }
	bool is_target_reachable() const { return path_target_reachable; }
};

} // namespace godot

VARIANT_ENUM_CAST(godot::AdvancedAINavigator3D::SteeringMode);
