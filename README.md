# C++ version of EdgeBoxes.

## Requirement

+ OpenCV 2.x

## Usage

```bash
$ git clone https://github.com/zimpha/EdgeBoxes.git
$ cd EdgeBoxes
$ mkdir build
$ cd build
$ cmake -B. -H..
$ make detect_demo
```

## Model

You can train the model using [pdollar/toolbox](https://github.com/pdollar/toolbox) and [pdollar/edges](https://github.com/pdollar/edges). Using the matlab code to convert the model. The model for ACF Detector is already converted.
