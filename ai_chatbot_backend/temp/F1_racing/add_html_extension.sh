#!/bin/bash
# Script to add .html extension to files without extensions
# Skips files that already have extensions and metadata files

TARGET_DIR="/home/bot/bot/yk/YK_final/courses/F1_racing"

find "$TARGET_DIR" -type f | while read -r file; do
    filename=$(basename "$file")

    # Skip if file already has an extension (contains a dot)
    if [[ "$filename" == *.* ]]; then
        continue
    fi

    # Skip hidden files
    if [[ "$filename" == .* ]]; then
        continue
    fi

    # Check if file is HTML content
    if head -c 100 "$file" 2>/dev/null | grep -qi "<!DOCTYPE\|<html"; then
        mv "$file" "$file.html"
        echo "Renamed: $file -> $file.html"
    fi
done

echo "Done!"
