// Host check that the firmware's C policy matches the trained PyTorch policy bit-for-bit (within float tolerance).
// Built and run by sim/test_firmware_policy.py: gcc -O2 -I../bleep test_policy.c -lm
#include <stdio.h>
#include <stdlib.h>
#include "policy.h"
int main(void) {
  float obs[POLICY_OBS], act[POLICY_ACT];
  while (1) {
    for (int i = 0; i < POLICY_OBS; i++) if (scanf("%f", &obs[i]) != 1) return 0;
    policy_forward(obs, act);
    for (int i = 0; i < POLICY_ACT; i++) printf("%.7g ", act[i]);
    printf("\n");
  }
}
