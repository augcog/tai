import pytest
import os
import time
import shutil
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, Future

from rag.file_conversion_router.utils.conversion_cache import ConversionCache
from rag.file_conversion_router.conversion.base_converter import BaseConverter
from rag.file_conversion_router.utils.utils import calculate_hash


# Test subclass of BaseConverter to test the cache functionality
class TestConverter(BaseConverter):
    """Test converter implementation for testing cache functionality"""

    def __init__(self, optimizer_config_path=None):
        super().__init__(optimizer_config_path=optimizer_config_path)

    def _to_markdown(self, input_path, output_path):
        """Simple implementation that just copies the file content to markdown"""
        with open(input_path, 'r') as f:
            content = f.read()

        with open(output_path, 'w') as f:
            f.write(f"# Converted from {input_path.name}\n\n{content}")

        # Create a pickle file as well (BaseConverter expects both files)
        with open(output_path.with_suffix('.pkl'), 'wb') as f:
            import pickle
            pickle.dump({"content": content}, f)

    def _to_page(self, input_path, output_path):
        """Simple implementation that returns a mock Page object"""
        return {"title": input_path.stem, "content": f"Content of {input_path.name}"}

    def _perform_conversion(self, input_path, output_folder):
        """Perform the actual conversion"""
        start_time = time.time()
        self._convert_to_markdown(input_path, self._md_path)
        conversion_time = time.time() - start_time
        return None, conversion_time


# Test fixtures
@pytest.fixture
def reset_cache():
    """Reset the cache before and after each test"""
    # Clear the cache
    ConversionCache._cache = {}
    ConversionCache._futures_cache = {}
    ConversionCache._times_cache = {}
    ConversionCache._access_count = {}
    ConversionCache._cache_file_path = None
    yield
    # Clean up after test
    ConversionCache._cache = {}
    ConversionCache._futures_cache = {}
    ConversionCache._times_cache = {}
    ConversionCache._access_count = {}
    ConversionCache._cache_file_path = None


@pytest.fixture
def test_files(tmp_path):
    """Create test files for testing"""
    input_dir = tmp_path / "input"
    input_dir.mkdir()

    # Create test files
    file1 = input_dir / "file1.txt"
    file2 = input_dir / "file2.txt"
    file3 = input_dir / "subfolder" / "file3.txt"

    file1.parent.mkdir(parents=True, exist_ok=True)
    file2.parent.mkdir(parents=True, exist_ok=True)
    file3.parent.mkdir(parents=True, exist_ok=True)

    # Write content to files
    with open(file1, 'w') as f:
        f.write("Content of file1")

    with open(file2, 'w') as f:
        f.write("Content of file2")

    with open(file3, 'w') as f:
        f.write("Content of file3")

    return input_dir, [file1, file2, file3]


@pytest.fixture
def cache_dir(tmp_path):
    """Create a cache directory"""
    cache_dir = tmp_path / "cache"
    cache_dir.mkdir()
    cache_file = cache_dir / "conversion_cache.pkl"
    return cache_file


class TestConversionCacheWithConverter:

    def test_cache_repeated_files(self, test_files, reset_cache, tmp_path):
        """Test that repeated conversions of the same file use the cache"""
        input_dir, files = test_files
        file1 = files[0]

        # Create output folders
        output1 = tmp_path / "output1"
        output2 = tmp_path / "output2"
        output1.mkdir()
        output2.mkdir()

        # Create converter
        converter = TestConverter()

        # First conversion
        converter.convert(file1, output1)

        # Get the file hash
        file_hash = calculate_hash(file1)

        # Check that the file is in the cache
        assert file_hash in ConversionCache._cache

        # Get the cached paths
        cached_paths = ConversionCache.get_cached_paths(file_hash)
        assert cached_paths is not None
        assert len(cached_paths) == 2  # md and pkl files

        # Second conversion
        converter.convert(file1, output2)

        # Check that files in output2 are copies of files in cache
        md_file1 = output1 / f"{file1.stem}.md"
        md_file2 = output2 / f"{file1.stem}.md"
        pkl_file1 = output1 / f"{file1.stem}.pkl"
        pkl_file2 = output2 / f"{file1.stem}.pkl"

        assert md_file1.exists()
        assert md_file2.exists()
        assert pkl_file1.exists()
        assert pkl_file2.exists()

        # Check content is the same
        with open(md_file1, 'r') as f1, open(md_file2, 'r') as f2:
            assert f1.read() == f2.read()

        # Check cache access count
        assert ConversionCache.get_access_count(file_hash) == 2

    def test_persistent_cache(self, test_files, reset_cache, cache_dir, tmp_path):
        """Test that persistent cache works across different runs"""
        input_dir, files = test_files
        file1 = files[0]

        # Create output folders
        output1 = tmp_path / "output1"
        output2 = tmp_path / "output2"
        output1.mkdir()
        output2.mkdir()

        # Set up cache file
        ConversionCache.set_cache_path(cache_dir)

        # Create converter
        converter = TestConverter()

        # First conversion
        converter.convert(file1, output1)

        # Get the file hash
        file_hash = calculate_hash(file1)

        # Check that the file is in the cache
        assert file_hash in ConversionCache._cache

        # Reset cache to simulate new run
        ConversionCache._cache = {}
        ConversionCache._times_cache = {}
        ConversionCache._access_count = {}

        # Set up cache file again
        ConversionCache.set_cache_path(cache_dir)

        # Second conversion (should use cached files)
        converter.convert(file1, output2)

        # Check that files in output2 exist
        md_file2 = output2 / f"{file1.stem}.md"
        pkl_file2 = output2 / f"{file1.stem}.pkl"

        assert md_file2.exists()
        assert pkl_file2.exists()

    def test_modified_files(self, test_files, reset_cache, tmp_path):
        """Test that cache handles modified files correctly"""
        input_dir, files = test_files
        file1 = files[0]

        # Create output folders
        output1 = tmp_path / "output1"
        output2 = tmp_path / "output2"
        output1.mkdir()
        output2.mkdir()

        # Create converter
        converter = TestConverter()

        # First conversion
        converter.convert(file1, output1)

        # Get the file hash before modification
        file_hash_before = calculate_hash(file1)

        # Modify the file
        time.sleep(0.1)  # Ensure modification time is different
        with open(file1, 'w') as f:
            f.write("Modified content of file1")

        # Get the file hash after modification
        file_hash_after = calculate_hash(file1)

        # Confirm hash changed
        assert file_hash_before != file_hash_after

        # Second conversion (should not use cached files due to different hash)
        converter.convert(file1, output2)

        # Check that files in output2 have different content
        md_file1 = output1 / f"{file1.stem}.md"
        md_file2 = output2 / f"{file1.stem}.md"

        with open(md_file1, 'r') as f1, open(md_file2, 'r') as f2:
            content1 = f1.read()
            content2 = f2.read()
            assert content1 != content2
            assert "Modified content" in content2

    def test_cache_with_multiple_files(self, test_files, reset_cache, tmp_path):
        """Test caching with multiple files"""
        input_dir, files = test_files

        # Create output folder
        output_dir = tmp_path / "output"
        output_dir.mkdir()

        # Create converter
        converter = TestConverter()

        # Convert all files
        for file in files:
            converter.convert(file, output_dir)

        # Check all files are in cache
        for file in files:
            file_hash = calculate_hash(file)
            assert file_hash in ConversionCache._cache

            # Check output files exist
            md_file = output_dir / f"{file.stem}.md"
            pkl_file = output_dir / f"{file.stem}.pkl"
            assert md_file.exists()
            assert pkl_file.exists()

        # Create new output folder
        output_dir2 = tmp_path / "output2"
        output_dir2.mkdir()

        # Convert all files again
        for file in files:
            converter.convert(file, output_dir2)

        # Check all files are copied to new output folder
        for file in files:
            md_file = output_dir2 / f"{file.stem}.md"
            pkl_file = output_dir2 / f"{file.stem}.pkl"
            assert md_file.exists()
            assert pkl_file.exists()

            # Check access count
            file_hash = calculate_hash(file)
            assert ConversionCache.get_access_count(file_hash) == 1

    def test_version_handling(self, test_files, reset_cache, tmp_path, monkeypatch):
        """Test that cache handles different versions correctly"""
        input_dir, files = test_files
        file1 = files[0]

        # Create output folders
        output1 = tmp_path / "output1"
        output2 = tmp_path / "output2"
        output1.mkdir()
        output2.mkdir()

        # Create converter
        converter = TestConverter()

        # First conversion with version 1.0
        original_version = ConversionCache.version
        ConversionCache.version = "1.0"
        converter.convert(file1, output1)

        # Get the file hash
        file_hash = calculate_hash(file1)

        # Add version to cache entry manually
        ConversionCache._cache[file_hash]["conversion_version"] = "1.0"

        # Change version to 2.0
        ConversionCache.version = "2.0"

        # Second conversion (should not use cached files due to version mismatch)
        # We'll patch the check_version method to simulate version checking
        def mock_check_version(self, file_hash):
            return False

        monkeypatch.setattr(ConversionCache, "check_version", mock_check_version)

        # This should force a new conversion due to version mismatch
        converter.convert(file1, output2)

        # Restore original version
        ConversionCache.version = original_version

        # Check that both files exist
        md_file1 = output1 / f"{file1.stem}.md"
        md_file2 = output2 / f"{file1.stem}.md"

        assert md_file1.exists()
        assert md_file2.exists()

    def test_parallel_processing(self, test_files, reset_cache, tmp_path):
        """Test that cache handles parallel processing correctly"""
        input_dir, files = test_files
        file1 = files[0]

        # Create output folders
        output1 = tmp_path / "output1"
        output2 = tmp_path / "output2"
        output1.mkdir()
        output2.mkdir()

        # Create converter
        converter1 = TestConverter()
        converter2 = TestConverter()

        # Get the file hash
        file_hash = calculate_hash(file1)

        # Create a future to simulate an in-progress conversion
        future = Future()
        ConversionCache.store_future(file_hash, future)

        # Start one conversion
        def delayed_conversion():
            time.sleep(0.2)  # Simulate a delay
            converter1.convert(file1, output1)
            return "Conversion done"

        with ThreadPoolExecutor() as executor:
            future_result = executor.submit(delayed_conversion)

            # Start another conversion immediately
            converter2.convert(file1, output2)

            # Wait for the first conversion to complete
            future_result.result()

        # Check that both files exist
        md_file1 = output1 / f"{file1.stem}.md"
        md_file2 = output2 / f"{file1.stem}.md"

        assert md_file1.exists()
        assert md_file2.exists()

        # Check that the cache has been updated
        assert file_hash in ConversionCache._cache

        # Get the cached paths
        cached_paths = ConversionCache.get_cached_paths(file_hash)
        assert cached_paths is not None

    def test_calc_total_savings(self, test_files, reset_cache, tmp_path):
        """Test that total time savings are calculated correctly"""
        input_dir, files = test_files

        # Create output folders
        output1 = tmp_path / "output1"
        output2 = tmp_path / "output2"
        output1.mkdir()
        output2.mkdir()

        # Create converter
        converter = TestConverter()

        # Convert all files
        for file in files:
            converter.convert(file, output1)

        # Convert all files again
        for file in files:
            converter.convert(file, output2)

        # Check total savings
        total_savings = ConversionCache.calc_total_savings()
        assert total_savings > 0  # Should have some savings from cached conversions