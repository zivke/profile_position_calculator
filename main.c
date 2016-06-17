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

        // TODO: The case when the target position is smaller than the MAX_ACCELERATION
        while(1) {
            calculated_steps[i] = init_position_profile(&motion_profiles[i], target_positions[i], start_positions[i], profile_velocity[i], \
                                                        acceleration, deceleration);
            if (calculated_steps[i] > max_steps) {
                // Make sure that returning to a previous position works as well
                int profile_velocity_correction;
                if (profile_velocity[i] > 0) {
                    profile_velocity_correction = profile_velocity[i]++;
                } else {
                    profile_velocity_correction = profile_velocity[i]--;
                }

                calculated_steps[i] = init_position_profile(&motion_profiles[i], target_positions[i], start_positions[i], \
                                                            profile_velocity_correction, acceleration, deceleration);
                break;
            }

            // Make sure that returning to a previous position works as well
            if (profile_velocity[i] > 0) {
                profile_velocity[i]--;
            } else {
                profile_velocity[i]++;
            }
        }
    }

    return motion_profiles;
}

// Calculate the profile position sections and APPEND them to the result file
void connect_positions(int *target_positions, int *start_positions, int n_axis, int gripper) {
    motion_profile_t *motion_profiles;

    motion_profiles = init_position_profile_multi_axis(target_positions, start_positions, n_axis, \
                                                       USER_PROFILE_VELOCITY, USER_PROFILE_ACCELERATION, USER_PROFILE_DECELERATION);

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

    FILE *file = fopen("calculated_positions.csv", "a");
    if (file == NULL)
    {
        printf("Error opening output file!\n");
        exit(1);
    }

    for (int i = 0; i <= motion_profiles[highest_offset_motor_index].steps; i++) {
        for (int j = 0; j < n_axis; j++) {
            int position = position_profile_generate(&motion_profiles[j], i);
            if (j + 1 < n_axis) {
                fprintf(file, "%d,",position);
            } else {
                // Finish the line with the gripper value
                fprintf(file, "%d,%d\n", position, gripper);
            }
        }
    }

    fclose(file);
}

int main(int argc, char **argv) {

    // The first argument is the input file name (CSV format with commas)
    FILE *file = fopen(argv[1], "r");
    if (file == NULL) {
        printf("Error opening input file!\n");
        exit(1);
    }

    // Count all of the target points in the input file (by counting rows)
    char c;
    int n_rows = 0;
    while ((c = getc(file)) != EOF) {
        if (c == '\n') {
            n_rows++;
        }
    }
    // Reset the file pointer to the beginning of the file
    rewind(file);

    // Create the 2D array to hold all of the read data from the input file
    int **array = malloc(n_rows * sizeof *array + (n_rows * (TOTAL_MOTORS * sizeof **array)));
    int *data = array + n_rows;
    for(int i = 0; i < n_rows; i++) {
        array[i] = data + i * TOTAL_MOTORS;
    }

    int gripper = 0;

    // Read each line of data and put in its place (can read exactly 6 positions and the gripper value)
    int line = 0;
    while(!feof(file)) {
        fscanf(file, "%d,%d,%d,%d,%d,%d,%d\n", &array[line][0], &array[line][1], &array[line][2], &array[line][3], \
                                               &array[line][4], &array[line][5], &gripper);
        line++;
    }

    fclose(file);

    file = fopen("calculated_positions.csv", "w");
    if (file == NULL)
    {
        printf("Error opening output file!\n");
        exit(1);
    }

    // Calculate the profile position sections and APPEND them to the result file
    for (int i = 0; i < n_rows; i++) {
        connect_positions(array[i+1], array[i], TOTAL_MOTORS, gripper);
    }

    fclose(file);

    return 0;
}
