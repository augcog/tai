'use client'
import * as React from 'react'
import { useState } from 'react'
import {
  DropdownMenu,
  DropdownMenuTrigger,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuSeparator
} from '@/tai/components/ui/dropdown-menu'
import { IconCaretDown } from '@/tai/components/ui/icons'
import { Button } from '@/tai/components/ui/button'
import { saveData } from '@/tai/lib/utils'

export function SelectCourse() {
  const [selectedCourse, setSelectedCourse] = useState('Public Domain Server') // Default message

  const handleSelect = (courseName: string) => {
    setSelectedCourse(courseName) // Update the selected course state
    saveData('selectedCourse', courseName) // Save the selected course to localStorage
    console.log('selectedCourse: ', courseName)
  }

  return (
    <DropdownMenu>
      <DropdownMenuTrigger asChild>
        <Button>
          {selectedCourse}
          <IconCaretDown />
        </Button>
      </DropdownMenuTrigger>
      <DropdownMenuContent>
        {/* Call handleSelect with the respective course name on click */}
        <DropdownMenuItem onSelect={() => handleSelect('CS 61A')}>
          CS 61A
        </DropdownMenuItem>
        <DropdownMenuItem onSelect={() => handleSelect('EE 106B')}>
          EE 106B
        </DropdownMenuItem>
        <DropdownMenuSeparator />
        <DropdownMenuItem onSelect={() => handleSelect('Public Domain Server')}>
          Public Domain Server
        </DropdownMenuItem>
      </DropdownMenuContent>
    </DropdownMenu>
  )
}
