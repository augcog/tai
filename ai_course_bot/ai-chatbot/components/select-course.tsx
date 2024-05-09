'use client';
import * as React from 'react';
import { useState } from 'react';
import { DropdownMenu, DropdownMenuTrigger, DropdownMenuContent, DropdownMenuItem, DropdownMenuSeparator } from '@/components/ui/dropdown-menu';
import { IconCaretDown } from '@/components/ui/icons';
import { Button } from '@/components/ui/button';
import { saveData } from '@/lib/utils';

export function SelectCourse() {
    const [selectedCourse, setSelectedCourse] = useState('Please Select A Course'); // Default message

    const handleSelect = (courseName: string) => {
        setSelectedCourse(courseName); // Update the selected course state
        saveData('selectedCourse', courseName); // Save the selected course to localStorage
        console.log("selectedCourse: ", courseName)
    };

    return (
        <DropdownMenu>
            <DropdownMenuTrigger>
                <Button>
                    {selectedCourse} {/* Use the selectedCourse state here */}
                    <IconCaretDown />
                </Button>
            </DropdownMenuTrigger>
            <DropdownMenuContent>
                {/* Call handleSelect with the respective course name on click */}
                <DropdownMenuItem onSelect={() => handleSelect('EE 106a')}>EE 106a</DropdownMenuItem>
                <DropdownMenuItem onSelect={() => handleSelect('Data 8')}>Data 8</DropdownMenuItem>
                <DropdownMenuSeparator />
                <DropdownMenuItem onSelect={() => handleSelect('General')}>General</DropdownMenuItem>
            </DropdownMenuContent>
        </DropdownMenu>
    );
}
