import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="fabrik_chain_3d",
    version="1.2.0",
    author="Atieh Merikh Nejadasl",
    author_email="atieh.merikh.nejadasl@vub.be",
    description="It is an implementation of the FABRIK‌ method for any 3D chain",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/Atiehmerikh/FABRIK_chain_3D",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)