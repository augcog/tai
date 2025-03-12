from fastapi import status


def test_get_course_files_valid_course(client_unit):
    """
    Test that a valid course returns a list of files with complete attributes.
    """
    response = client_unit.get("/v1/courses/CS61A/files?page=1&limit=10")
    assert response.status_code == 200
    data = response.json()
    assert "data" in data and "meta" in data
    files = data["data"]
    assert isinstance(files, list)
    if files:
        file = files[0]
        for key in ["fileId", "name", "isDirectory", "path", "updatedAt", "size", "fileType"]:
            assert key in file


def test_get_course_files_invalid_course(client_unit):
    """
    Test that requesting files for an invalid course returns 404.
    """
    response = client_unit.get("/v1/courses/INVALID/files?page=1&limit=10")
    assert response.status_code == status.HTTP_404_NOT_FOUND
    data = response.json()
    assert "detail" in data
    assert data["detail"] == "Course not found"


def test_get_file_detail_valid(client_unit):
    """
    Test retrieving file details for a valid file.
    """
    response = client_unit.get("/v1/files/file123")
    assert response.status_code == 200
    data = response.json()
    for key in ["fileId", "name", "url", "metaData"]:
        assert key in data
    assert data["fileId"] == "file123"


def test_get_file_detail_invalid(client_unit):
    """
    Test retrieving file details for an invalid file returns a proper error.
    """
    response = client_unit.get("/v1/files/invalid_file")
    # Depending on your implementation, this might return 404 or a JSON null.
    assert response.status_code == status.HTTP_404_NOT_FOUND or response.json() is None
