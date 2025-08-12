import os
import random
import tarfile
from dataclasses import dataclass
from urllib.parse import urljoin

from bs4 import BeautifulSoup

import requests

from ...orm.ormatic_interface import *


@dataclass
class ProcTHORInterface:
    """
    Interface for interacting with the ProcThor environments.
    This class provides methods to scrape, download, and extract .tar.gz files containing ProcThor environments.
    Base URL defaults to 'https://user.informatik.uni-bremen.de/~luc_kro/procthor_environments/'
    """

    base_url: str = (
        "https://user.informatik.uni-bremen.de/~luc_kro/procthor_environments/"
    )
    """
    The base URL to scrape for .tar.gz files containing ProcThor environments.
    """

    project_root: str = os.path.join(os.path.expanduser("~"), "semantic_world")
    """
    The root path of the project, used to find resources and other files.
    """

    def get_tarball_links(self) -> List[str]:
        """
        Scrape and return all .tar.gz file links from the specified base URL.

        :return: A list of URLs pointing to .tar.gz files.
        """
        response = requests.get(self.base_url)
        soup = BeautifulSoup(response.text, "html.parser")
        return [
            urljoin(self.base_url, a["href"])
            for a in soup.find_all("a", href=True)
            if a["href"].endswith(".tar.gz")
        ]

    def download_file(self, url: str, filename: str):
        """
        Download a file from a URL to a destination path.

        :param url: The URL of the file to download.
        :param filename: The name of the file to save as.
        """
        response = requests.get(url, stream=True)
        tmp_folder = os.path.join(self.project_root, "tmp")
        os.makedirs(tmp_folder, exist_ok=True)
        with open(os.path.join(tmp_folder, filename), "wb") as f:
            for chunk in response.iter_content(chunk_size=8192):
                if chunk:
                    f.write(chunk)

    def extract_tar(self, filename: str, extract_to: str):
        """
        Extract a .tar.gz archive to the specified directory.

        :param filename: The name of the .tar.gz file to extract.
        :param extract_to: Directory where the contents should be extracted.
        """
        os.makedirs(extract_to, exist_ok=True)
        mode = "r:gz" if filename.endswith(".gz") else "r:"
        with tarfile.open(
            os.path.join(self.project_root, "tmp", filename), mode
        ) as tar:
            tar.extractall(path=extract_to)

    def replace_mesh_paths(self, filename: str, environment_name: str):
        # Read in the file
        with open(filename, "r") as file:
            filedata = file.read()

        # replace mesh paths without file
        filedata = filedata.replace(
            f'filename="file://{environment_name}/',
            f'filename="file://resources/procthor_environments/{environment_name}/{environment_name}/',
        )

        # Replace the target string
        filedata = filedata.replace("file://", f"{os.path.abspath(self.project_root)}/")
        # Write the file out again
        with open(filename, "w") as file:
            file.write(filedata)

    def sample_environment(
        self, house_number: Optional[int] = None, keep_environment: bool = False
    ):
        """
        Fetch and extract a random selection of environments packed in .tar.gz files from a URL.

        :param keep_environment: If True, the environments will be kept in the resources directory, otherwise they will be
        """
        if not keep_environment:
            output_dir = os.path.join(self.project_root, "tmp")
        else:
            output_dir = os.path.join(
                self.project_root, "resources/procthor_environments"
            )
        if house_number is None:
            links = self.get_tarball_links()
            selected_link = random.choice(links)
            filename = os.path.basename(selected_link)
        else:
            filename = f"dataset_house_{house_number}.tar.gz"
        environment_name = os.path.splitext(os.path.splitext(filename)[0])[0]
        self.download_file(urljoin(self.base_url, filename), filename)
        self.extract_tar(filename, output_dir)
        os.remove(os.path.join(self.project_root, "tmp", filename))
        # self.replace_mesh_paths(os.path.join(self.project_root, "resources", "procthor_environments",
        #                                      environment_name, f"{environment_name}.urdf"))
        self.replace_mesh_paths(
            os.path.join(
                self.project_root,
                "resources",
                "procthor_environments",
                environment_name,
                f"{environment_name}.urdf",
            ),
            environment_name
        )
        return output_dir, environment_name
