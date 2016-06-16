#include <stdio.h>
#include <stdlib.h>
#include "profile.h"

#define TOTAL_MOTORS 6
#define MAX_ACCELERATION 5000
#define MAX_VELOCITY 4500
#define MAX_POSITION 2000000000
#define MIN_POSITION -2000000000

#define USER_PROFILE_VELOCITY 3000
#define USER_PROFILE_ACCELERATION 4000
#define USER_PROFILE_DECELERATION 4000

motion_profile_t * init_position_profile_multi_axis(int *target_positions, int *start_positions, int n_axis, int velocity, int acceleration, int deceleration) {

    motion_profile_t *motion_profiles = malloc(n_axis * sizeof(motion_profile_t));

    int *calculated_steps = malloc(n_axis * sizeof(int));
    int *profile_velocity = malloc(n_axis * sizeof(int));

    int delta_max = 0;
    int highest_offset_motor_index = 0;

    // Find highest position offset motor index
    for (int i = 0; i < n_axis; i++) {
        int delta = target_positions[i] - start_positions[i];
        if (delta > delta_max) {
            delta_max = delta;
            highest_offset_motor_index = i;
        }
    }

    init_position_profile_limits(&motion_profiles[highest_offset_motor_index], MAX_ACCELERATION, MAX_VELOCITY, MAX_POSITION, MIN_POSITION);

    int max_steps = init_position_profile(&motion_profiles[highest_offset_motor_index], target_positions[highest_offset_motor_index], start_positions[highest_offset_motor_index], \
                                          velocity, acceleration, deceleration);

    for (int i = 0; i < n_axis; i++) {
        if (highest_offset_motor_index == i) {
            profile_velocity[i] = velocity;
            calculated_steps[i] = max_steps;
            continue;
        }

        calculated_steps[i] = 0;
        profile_velocity[i] = velocity;

        init_position_profile_limits(&motion_profiles[i], MAX_ACCELERATION, MAX_VELOCITY, MAX_POSITION, MIN_POSITION);

        while(1) {
            calculated_steps[i] = init_position_profile(&motion_profiles[i], target_positions[i], start_positions[i], profile_velocity[i], \
                                                        acceleration, deceleration);
            if (calculated_steps[i] > max_steps) {
                calculated_steps[i] = init_position_profile(&motion_profiles[i], target_positions[i], start_positions[i], ++profile_velocity[i], \
                                                            acceleration, deceleration);
                break;
            }
            profile_velocity[i]--;
        }
    }

    return motion_profiles;
}

int main () {

    int start_positions[TOTAL_MOTORS] = { 0, 0, 0, 0, 0, 0 };
    int target_positions[TOTAL_MOTORS] = { 100000, 200000, 300000, 400000, 500000, 600000 };

    motion_profile_t *motion_profiles;

    motion_profiles = init_position_profile_multi_axis(target_positions, start_positions, TOTAL_MOTORS, \
                                                       USER_PROFILE_VELOCITY, USER_PROFILE_ACCELERATION, USER_PROFILE_DECELERATION);

    int delta_max = 0;
    int highest_offset_motor_index = 0;

    // Find highest position offset motor index
    for (int i = 0; i < TOTAL_MOTORS; i++) {
        int delta = target_positions[i] - start_positions[i];
        if (delta > delta_max) {
            delta_max = delta;
            highest_offset_motor_index = i;
        }
    }

    FILE *f = fopen("positions.csv", "w");
    if (f == NULL)
    {
        printf("Error opening file!\n");
        exit(1);
    }

    for (int i = 0; i <= motion_profiles[highest_offset_motor_index].steps; i++) {
        for (int j =0; j < TOTAL_MOTORS; j++) {
            int position = position_profile_generate(&motion_profiles[j], i);
            fprintf(f, "%d,",position);
        }
        fprintf(f, "0\n");
    }

    fclose(f);

    return 0;
}
