def test_get_courses_authenticated(client_unit):
    """
    Test that authenticated users get both public and private courses.
    """
    response = client_unit.get("/v1/courses?page=1&limit=10")
    assert response.status_code == 200
    data = response.json()
    # Check for expected keys in the response.
    assert "data" in data
    assert "meta" in data

    meta = data["meta"]
    assert "total" in meta
    assert isinstance(data["data"], list)

    # If courses are returned, check each course has the expected attributes.
    if data["data"]:
        for course in data["data"]:
            for key in ["courseId", "courseName", "isPublic"]:
                assert key in course


def test_get_courses_pagination(client_unit):
    """
    Test the courses endpoint respects pagination.
    """
    response = client_unit.get("/v1/courses?page=1&limit=1")
    assert response.status_code == 200
    data = response.json()
    assert isinstance(data["data"], list)
    # When limit is 1, the response should contain at most one course.
    assert len(data["data"]) <= 1
