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
import { saveData, loadData } from '@/tai/lib/utils'

export function SelectCourse() {
  const [selectedCourse, setSelectedCourse] = useState(
    loadData('selectedCourse')
  )

  const handleSelect = (courseName: string) => {
    setSelectedCourse(courseName)
    saveData('selectedCourse', courseName)
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
        <DropdownMenuItem onSelect={() => handleSelect('Econ 140')}>
          Econ 140
        </DropdownMenuItem>
        <DropdownMenuItem onSelect={() => handleSelect('CS 294-137')}>
          CS 294-137
        </DropdownMenuItem>
        <DropdownMenuSeparator />
        <DropdownMenuItem onSelect={() => handleSelect('Public Domain Server')}>
          Public Domain Server
        </DropdownMenuItem>
      </DropdownMenuContent>
    </DropdownMenu>
  )
}
