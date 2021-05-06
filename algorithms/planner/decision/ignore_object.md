# Ignore object design doc

- In some cases, FOD or VEGETATION may cause unexpected hard brake. To ensure the comfort of driving behavior, ignore them.

## Workflow

1. Only ignore objects with type OT_FOD or OT_VEGETATION.
2. Objects marked "Ignore" if:

   - We can't stop before them. [MR-7272](https://gitlab.qcraft.ai/root/qcraft/-/merge_requests/7272)
