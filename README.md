# rustrol

A control algorithm demonstration project in Rust

Try it now on your browser!

https://msakuta.github.io/rustrol/


## What is this?

This is a demonstration repository for control problems that is solved with [rustograd](https://github.com/msakuta/rustograd), an automatic differentiation library.
It has native GUI and WebAssembly version that can run on a browser.

![screenshot](images/screenshot.png)

There are 2 models:

* Lunar Lander
* Missile

### Lunar Lander

![screencapture](images/screencapture.gif)

This is the same problem as the Lunar Lander game presented in [gym](https://www.gymlibrary.dev/content/basic_usage/) Python library for developing AI.
The gym is targetted for reinforcement learning, but actually we can do some decent job by just using the gradient descent on a cost function and its automatic differentiation.

![moon-lander animation](https://user-images.githubusercontent.com/15806078/153222406-af5ce6f0-4696-4a24-a683-46ad4939170c.gif)


### Missile

A homing missile maneuver with gravity and limited maneuverability.
It is a little bit more challenging because both the target and the missile are moving.
However, it is very easy to define a loss function as the minimum distance between target and missile in every moment.

![screencapture_missile](images/screencapture_missile.gif)


## Control

Clicking on the canvas will start the lander vehicle from clicked position.

You can also try to maneuver the lander by yourself if you check `direct_control`.
You can use A and D keys to change orientation of the lander and press W to thrust upwards (relative to current lander's orientation).


## How to run natively

```
cargo r
```


## How to build a WebAssembly version

Install trunk by 

```
cargo install --locked trunk
```

and run

```
trunk build --release
```

