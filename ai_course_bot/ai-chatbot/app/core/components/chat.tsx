'use client'

import { useChat, type Message } from 'ai/react'

import { cn } from '@/tai/lib/utils'
import { ChatList } from '@/tai/components/chat-list'
import { ChatPanel } from '@/tai/components/chat-panel'
import { EmptyScreen } from '@/tai/components/empty-screen'
import { ChatScrollAnchor } from '@/tai/components/chat-scroll-anchor'
import { useLocalStorage } from '@/tai/lib/hooks/use-local-storage'
import { toast } from 'react-hot-toast'
import { usePathname, useRouter } from 'next/navigation'
import { SelectCourse } from '@/tai/components/select-course'
import { useSession } from 'next-auth/react'
import emitter from '@/tai/utils/eventEmitter'

const IS_PREVIEW = process.env.VERCEL_ENV === 'preview'
export interface ChatProps extends React.ComponentProps<'div'> {
  initialMessages?: Message[]
  id?: string
}

export function Chat({ id, initialMessages, className }: ChatProps) {
  const { data: session, status } = useSession()

  const router = useRouter()
  const path = usePathname()
  const [previewToken, setPreviewToken] = useLocalStorage<string | null>(
    'ai-token',
    null
  )

  const saveChat = async (
    prev_messages: Message[],
    input: string,
    assistant: Message,
    id: string | undefined
  ) => {
    try {
      let messages = prev_messages.map(message => {
        return {
          role: message.role,
          content: message.content
        }
      })

      messages.push({
        role: 'user',
        content: input
      })

      messages.push({
        role: 'assistant',
        content: assistant.content
      })

      const response = await fetch('/api/chat/save', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          id,
          messages
        })
      })

      if (!response.ok) {
        throw new Error('Failed to save chat')
      }

      if (!path.includes('chat')) {
        window.history.replaceState({}, '', `/chat/${id}`)
        router.replace(`/chat/${id}`)
        setTimeout(() => {
          emitter.emit('historyUpdated')
        }, 2000)
      }
    } catch (error) {
      console.error('Error saving chat:', error)
    }
  }

  const { messages, append, reload, stop, isLoading, input, setInput } =
    useChat({
      api: '/api/chat',
      initialMessages,
      id,
      body: {
        id,
        previewToken
      },
      onResponse(response) {
        if (response.status === 401) {
          toast.error(response.statusText)
        }
      },
      onFinish(message: Message) {
        saveChat(messages, input, message, id)
      }
    })
  return (
    <>
      <div className={cn('pb-[200px] pt-4 md:pt-10', className)}>
        <div
          style={{ paddingLeft: 10, display: 'inline-block', float: 'left' }}
        >
          {status === 'authenticated' && <SelectCourse />}
        </div>

        {messages.length ? (
          <>
            <ChatList messages={messages} />
            <ChatScrollAnchor trackVisibility={isLoading} />
          </>
        ) : (
          <EmptyScreen setInput={setInput} />
        )}
      </div>
      <ChatPanel
        id={id}
        isLoading={isLoading}
        stop={stop}
        append={append}
        reload={reload}
        messages={messages}
        input={input}
        setInput={setInput}
      />
    </>
  )
}
