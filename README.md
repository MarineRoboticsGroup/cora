# CORA++

This is an efficient C++ implementation of the certifiably correct range-aided
SLAM algorithm (CORA). Please see the [paper](https://arxiv.org/abs/2302.11614)
for more details.

## Usage

*TODO*: write up usage after we have a working version.

## Contributing

Any contributions should pass all checks in our `.pre-commit-config.yaml` file.
To install the pre-commit hooks, run `pre-commit install` in the root directory
of this repository.

You may need to install some dependencies to get the pre-commit hooks to work.

```
sudo apt-get install cppcheck
pip install cpplint
```




## Standing on the Shoulders of Giants

This implementation largely follows the structure of the
[SE-Sync C++ implementation](https://github.com/david-m-rosen/SE-Sync).


## Contributors

* [Tim Magoun](https://www.linkedin.com/in/timmagoun/)
* [Alan Papalia](https://alanpapalia.github.io/)


## References

If you use this code in your research, please cite the following paper:

```
@article{papalia2023certifiably,
  title={Certifiably Correct Range-Aided SLAM},
  author={Papalia, Alan and Fishberg, Andrew and O'Neill, Brendan W. and How, Jonathan P. and Rosen, David M. and Leonard, John J.},
  journal={arXiv preprint arXiv:2302.11614},
  year={2023}
}
```
