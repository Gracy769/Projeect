# loco_autoresearch

This is an experiment to have an LLM do its own research to optimize a locomotive regenerative braking system.

## Setup

To set up a new experiment:
1. **Initialize results.tsv**: Create `results.tsv` with just the header row.
2. **Confirm and go**: Confirm setup looks good.

Once you get confirmation, kick off the experimentation.

## Experimentation

Each experiment runs a locomotive braking physics simulation. You launch it simply as: `python train.py`.

**What you CAN do:**
- Modify `train.py` — this is the only file you edit. The `braking_controller` function determines the target slip based on current speed and time. You can write complex control logic, state machines, use speed-dependent curves, etc.

**What you CANNOT do:**
- Modify `prepare.py`. It is read-only. It contains the fixed physics engine and evaluation logic.
- Modify the evaluation metric.

**The goal is simple: get the lowest braking_score.** 
The score is calculated as `stop_distance - regen_kwh * 20`. A lower (more negative) score means a shorter stopping distance and higher recovered energy. 
Everything is fair game: change how slip behaves at high speeds vs low speeds, add non-linear profiles, etc.

**Simplicity criterion**: All else being equal, simpler is better.

**The first run**: Your very first run should always be to establish the baseline, so you will run the `python train.py` script as is.

## Output format

Once the script finishes it prints a summary like this:

```
---
braking_score:    123.456789
stop_distance_m:  900.00
regen_kwh:        25.00
stop_time_s:      45.00
sim_seconds:      0.0123
```

You can extract the key metric from the log file:

```
grep "^braking_score:" run.log
```

## Logging results

When an experiment is done, log it to `results.tsv` (tab-separated, NOT comma-separated).

The TSV has a header row and 4 columns:

```
commit	braking_score	status	description
```

1. git commit hash or just a running index (e.g. 001, 002) if git is not used.
2. braking_score achieved
3. status: `keep`, `discard`, or `crash`
4. short text description of what this experiment tried

Example:
```
commit	braking_score	status	description
001	450.000	keep	baseline constant slip -0.12
002	448.500	keep	linear slip decrease with speed
003	460.000	discard	step function slip
```

## The experiment loop

LOOP FOREVER:

1. Tune `train.py` with an experimental control idea by directly hacking the code.
2. Run the experiment: `python train.py > run.log 2>&1`
3. Read out the results: `cat run.log`
4. Record the results in the tsv.
5. If braking_score improved (lower), you KEEP the changes.
6. If braking_score is equal or worse, you REVERT back to the previous best version of `train.py`.

**NEVER STOP**: Once the experiment loop has begun (after the initial setup), do NOT pause to ask the human if you should continue. The loop runs until the human interrupts you, period.
