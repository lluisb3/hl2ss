import os
from plyfile import PlyData, PlyProperty
import click


def ply_double_to_float(ply_path):
    plydata = PlyData.read(ply_path)

    #go through all property one by one and if it is a double, we change it to an equivalent property in float
    real_properties = []
    for i in plydata.elements[0].properties:
        if str(PlyProperty(i.name, "double")) == str(i):
            real_properties.append(PlyProperty(i.name, "float32"))
        else:
            real_properties.append(i)
    real_properties = tuple(real_properties)

    #Save the same ply file but with float properties instead of double
    plydata.elements[0].properties = real_properties

    #Write the data back to the original pointcloudt
    os.remove(ply_path)
    plydata.write(ply_path)


@click.command()
@click.option(
    "--ply_path",
    default="scene",
    prompt="Path to .ply file to convert double to float",
    help="Path to .ply file to convert double to float",
)
def main(ply_path):
    ply_double_to_float(ply_path)


if __name__=="__main__":
    main()
