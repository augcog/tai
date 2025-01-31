import { MongoClient, Db } from 'mongodb'

const uri = process.env.MONGO_URI as string

let client: MongoClient
let clientPromise: Promise<MongoClient>

if (process.env.NODE_ENV === 'development') {
  // @ts-ignore
  if (!global._mongoClientPromise) {
    client = new MongoClient(uri)
    // @ts-ignore
    global._mongoClientPromise = client.connect()
  }
  // @ts-ignore
  clientPromise = global._mongoClientPromise
} else {
  client = new MongoClient(uri)
  clientPromise = client.connect()
}

export async function connectToDatabase(): Promise<{
  db: Db
  client: MongoClient
}> {
  const client = await clientPromise
  const db = client.db('main')
  return { client, db }
}
