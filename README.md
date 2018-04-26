# Improved "Follow Me" Behavior

A RobAir that follows a person, locks onto them, and avoids obstacles.

## Team: Group 9

Mykhailo RIMEL (mykhailo.rimel@grenoble-inp.org)
Philip SCALES (philip.scales@etu.univ-grenoble-alpes.fr)
Branislav SMIK (branislav.smik@grenoble-inp.org)
Ming ZHANG (Ming.Zhang@grenoble-inp.org)

## How to run

```
rosrun follow_me launch.sh
```

## How to terminate

Press 'k' and Enter

## Setup
We assume that there are two lasers connected to the RobAIR:
the bottom laser publishes data to "/scan2" topic
the top laser publishes data to "/scan1" topic

It is also important that lasers are placed roughly in the same way we placed them
on the Base 2 (top laser is roughly 10 cm behind and 15 cm to the right of the bottom one)
because there is a hard-coded offset for the data reported by the top laser.

## Some general notes about the code
There are still several nodes that are not actually used anymore and kept as a part
of this archive just to make sure that it exactly matches the code that was demoed.

The only three nodes that are actually used are:
moving_persons_detector_node.cpp
decision_node.cpp
translation_action_node.cpp
