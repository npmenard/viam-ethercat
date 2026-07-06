from conan import ConanFile
from conan.tools.build import check_min_cppstd
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy
from conan.tools.scm import Git

class ethercat(ConanFile):
    name = "viam-ethercat"

    license = "Apache-2.0"
    url = "https://github.com/viam-modules/ethercat"
    package_type = "application"
    settings = "os", "compiler", "build_type", "arch"

    options = {
        "shared": [True, False]
    }

    default_options = {
        "shared": True
    }

    def export_sources(self):
        for pat in ["CMakeLists.txt", "cmake/*", "LICENSE", "src/*", "tests/*", "etc/*", "meta.json*", "*.sh", "README.md"]:
            copy(self, pat, self.recipe_folder, self.export_sources_folder)

    def set_version(self):
        git = Git(self)
        try:
            self.version = git.run("describe --tags --always").strip().lstrip('v')
        except Exception:
            self.version = "0.0.0-unknown"

    def requirements(self):
        # NOTE: If you update the `viam-cpp-sdk` dependency here, it should
        # also be updated in `bin/setup.sh` and the Dockerfile.
        self.requires("viam-cpp-sdk/[>=0.31.0]")
        self.requires("boost/[>=1.74.0]")
        # SOEM is NOT a Conan dependency: it is pinned by exact commit SHA and
        # fetched at configure time (see cmake/soem.cmake), so the recipe needs
        # network access during build. All SOEM knowledge stays in that file.

    def validate(self):
        check_min_cppstd(self, 20)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generate()
        CMakeDeps(self).generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def layout(self):
        cmake_layout(self, src_folder=".")

    def package(self):
        CMake(self).install()

        # Use CPack to build the module.tar.gz and manually copy it to the package folder
        CMake(self).build(target='package')
        copy(self, pattern="module.tar.gz", src=self.build_folder, dst=self.package_folder)

    def deploy(self):
        copy(self, pattern="module.tar.gz", src=self.package_folder, dst=self.deploy_folder)
