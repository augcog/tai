'use client'
import { clearChats, getChats } from '@/tai/utils/actions'
import { ClearHistory } from '@/tai/components/clear-history'
import { SidebarItems } from '@/tai/components/sidebar-items'
import { ThemeToggle } from '@/tai/components/theme-toggle'
import { useState, useEffect } from 'react'
import emitter from '@/tai/utils/eventEmitter'
import { type Chat } from '@/tai/lib/types'

interface SidebarListProps {
  userId?: string
  children?: React.ReactNode
}

export function SidebarList({ userId }: SidebarListProps) {
  const [chats, setChats] = useState<Chat[] | undefined>(undefined)

  const loadChats = async () => {
    try {
      const data = await getChats(userId)
      setChats(data)
    } catch (error) {
      console.error('Error fetching chats:', error)
    }
  }

  useEffect(() => {
    loadChats()

    const handleChatSaved = () => {
      loadChats()
    }

    emitter.on('historyUpdated', handleChatSaved)

    return () => {
      emitter.off('historyUpdated', handleChatSaved)
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [])

  return (
    <div className="flex flex-1 flex-col overflow-hidden">
      <div className="flex-1 overflow-auto">
        {chats === undefined ? null : chats.length > 0 ? (
          <div className="space-y-2 px-2">
            <SidebarItems chats={chats} />
          </div>
        ) : (
          <div className="p-8 text-center">
            <p className="text-sm text-muted-foreground">No chat history</p>
          </div>
        )}
      </div>
      <div className="flex items-center justify-between p-4">
        <ThemeToggle />
        <ClearHistory
          clearChats={clearChats}
          isEnabled={!!(chats?.length ?? 0)}
        />
      </div>
    </div>
  )
}
