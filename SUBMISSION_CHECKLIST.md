# Submission Checklist

Deadline from the assessment: **Tuesday, 7 July 2026 at 5:00 pm**.

## Current local validation status

Completed on 6 July 2026:

- [x] Corrected Docker image built successfully.
- [x] All 19 tests passed.
- [x] PX4 preflight, offboard, arming, 20 m takeoff, and follow mode succeeded.
- [x] Persistent JSONL telemetry and all four plots were generated under `runtime_logs/`.
- [x] Final-window CI validation passed.

Still required before sending the submission:

- [ ] Replace the personal and repository placeholders below.
- [ ] Push to a public GitHub repository.
- [ ] Confirm the GitHub Actions integration job passes from the public repository.
- [ ] Test a clean clone and send the completed email.

## 1. Replace the unavoidable personal placeholders

- Replace `<PUBLIC_GITHUB_REPOSITORY_URL>` in `README.md` and `EMAIL_SUBMISSION.txt`.
- Replace `meet@example.com` in `drone_system/package.xml`, `drone_system/setup.py`, and `car_motion_plugin/package.xml` with your real email address.
- Replace both availability placeholders in `EMAIL_SUBMISSION.txt` with two real 30-minute review slots and include the time zone.

A final placeholder scan must return no output:

```bash
grep -RInE '<PUBLIC_GITHUB_REPOSITORY_URL>|<OPTION_[12]_DATE_TIME_AND_TIME_ZONE>|meet@example.com' \
  --exclude='SUBMISSION_CHECKLIST.md' .
```

## 2. Run the real Docker integration test

```bash
docker build -t drone-car-follower .
rm -rf artifacts && mkdir -p artifacts
docker run --rm --init --network host --shm-size 1g \
  -v "$PWD/artifacts:/artifacts" \
  -e DRONE_SYSTEM_LOG=/artifacts/ci_run.jsonl \
  drone-car-follower \
  bash -lc '/workspace/src/drone-car-follower/tools/run_integration.sh 60'
```

Do not submit unless all of these are true:

- The command exits successfully.
- `artifacts/ci_run.jsonl` exists and `python3 tools/log_summary.py artifacts/ci_run.jsonl` is sensible.
- All four PNG files exist under `artifacts/plots/`.
- Drone altitude is above 1 m for the final 30 seconds and there are no final-window errors.
- The car stream remains active.
- `ANALYSIS.md` is updated if the real integration run reveals another limitation.

## 3. Publish the public repository

```bash
git init
git add .
git commit -m "Complete PX4 ROS 2 drone-car follower assessment"
git branch -M main
git remote add origin <PUBLIC_GITHUB_REPOSITORY_URL>
git push -u origin main
```

After pushing, test the reviewer path from a clean directory:

```bash
cd /tmp
rm -rf drone-car-follower-review
git clone <PUBLIC_GITHUB_REPOSITORY_URL> drone-car-follower-review
cd drone-car-follower-review
docker build -t drone-car-follower-review .
```

Confirm the GitHub Actions `integration-test` job is green and its `integration-logs` artifact contains the JSONL log and four plots. Also confirm the repository is accessible without credentials.

## 4. Submit

Send the completed text from `EMAIL_SUBMISSION.txt` to `info@invictron.in`. The required native entry point must remain exactly:

```bash
ros2 launch drone_system full_stack.launch.py
```
