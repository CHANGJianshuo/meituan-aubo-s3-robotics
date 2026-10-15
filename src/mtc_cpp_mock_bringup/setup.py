from setuptools import setup, find_packages
setup(name="mtc_cpp_mock_bringup", version="0.1.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/mtc_cpp_mock_bringup"]),
                  ("share/mtc_cpp_mock_bringup", ["package.xml", "README.md", "provenance.json"])],
      package_data={"mtc_webui": ["static/*", "provenance.json", "connection.example.json"]},
      entry_points={"console_scripts": [
        "mock_stack = mtc_cpp_mock_bringup.bringup:main",
        "mock_check = mtc_cpp_mock_bringup.check:main",
        "mock_task = mtc_cpp_mock_bringup.client:main",
        "web_console = mtc_webui.server:main"]})
